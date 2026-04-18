#include "takhandler.h"

#include <LittleFS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <mbedtls/error.h>
#include <mbedtls/pk.h>
#include <mbedtls/sha256.h>
#include <mbedtls/x509.h>
#include <mbedtls/x509_crt.h>
#include <time.h>
#include <math.h>
#include <TinyGPSPlus.h>

#include "filehandler.h"

extern Config config;
extern TinyGPSPlus gps;
extern String jsonDir;
extern String fileConfigJSON;

namespace {

WiFiClient takPlainClient;
WiFiClientSecure takSecureClient;
WiFiClient *takClient = &takPlainClient;

// Keep PEM buffers alive for as long as WiFiClientSecure may reference them.
String takLoadedCaPem;
String takLoadedClientCertPem;
String takLoadedClientKeyPem;

bool takConnected = false;
bool takCertExists = false;
bool takKeyExists = false;
bool takCAExists = false;
unsigned long takLastConnectAttemptMs = 0;
String takLastMessage = "Idle";
unsigned long takLastCotSentMs = 0;
unsigned long takLastCotAttemptMs = 0;
uint32_t takCotTxCount = 0;
uint32_t takCotTxFailCount = 0;

// Reconnection runtime state
unsigned long takLastDisconnectMs = 0;       // millis() when connection was lost; 0 = not in reconnect cycle
uint32_t takReconnectAttempts = 0;           // Attempts in the current disconnect cycle
unsigned long takReconnectCurrentDelayMs = 0;// Backoff delay currently in effect
unsigned long takReconnectNextAttemptMs = 0; // When to fire the next attempt
String takReconnectLastReason = "";          // Last reconnection failure reason
bool takReconnectGivenUp = false;            // True once max-duration timeout is exceeded
wl_status_t takPrevWifiStatus = WL_DISCONNECTED; // Tracks WiFi state changes
uint32_t takReconnectTotalAttempts = 0;      // Cumulative attempts across sessions (persisted)
uint32_t takReconnectTotalSuccesses = 0;     // Cumulative successes across sessions (persisted)
bool takManualConnectRequested = false;      // Manual connect queued from web UI
bool takManualConnectInProgress = false;     // Manual connect currently being processed in loop context

constexpr unsigned long kTakCotIntervalMs = 5000;
constexpr unsigned long kTakCotStaleSeconds = 120;
constexpr uint32_t kTakReconnectMaxAttemptsPerCycle = 3;

struct PemCheckResult {
  bool ok;
  int code;
  String details;
};

struct ParsedCertInfo {
  bool ok;
  String subject;
  String issuer;
  String serial;
  String sha256;
  String details;
};

String normalizePemText(const String &rawText) {
  String text = rawText;
  text.replace("\r\n", "\n");
  text.replace("\r", "\n");
  return text;
}

String extractFirstPemBlock(const String &rawText, const char *beginMarker, const char *endMarker) {
  String text = normalizePemText(rawText);
  int beginIdx = text.indexOf(beginMarker);
  if (beginIdx < 0) {
    return "";
  }

  int endIdx = text.indexOf(endMarker, beginIdx);
  if (endIdx < 0) {
    return "";
  }

  endIdx += strlen(endMarker);
  String block = text.substring(beginIdx, endIdx);
  block.trim();
  block += "\n";
  return block;
}

String extractAllCertificatePemBlocks(const String &rawText) {
  String text = normalizePemText(rawText);
  String output = "";
  const char *beginMarker = "-----BEGIN CERTIFICATE-----";
  const char *endMarker = "-----END CERTIFICATE-----";

  int searchStart = 0;
  while (true) {
    int beginIdx = text.indexOf(beginMarker, searchStart);
    if (beginIdx < 0) {
      break;
    }

    int endIdx = text.indexOf(endMarker, beginIdx);
    if (endIdx < 0) {
      break;
    }

    endIdx += strlen(endMarker);
    String block = text.substring(beginIdx, endIdx);
    block.trim();
    if (block.length() > 0) {
      output += block + "\n";
    }
    searchStart = endIdx;
  }

  return output;
}

String extractPrivateKeyPemBlock(const String &rawText) {
  String key = extractFirstPemBlock(rawText, "-----BEGIN PRIVATE KEY-----", "-----END PRIVATE KEY-----");
  if (key.length() > 0) {
    return key;
  }

  key = extractFirstPemBlock(rawText, "-----BEGIN RSA PRIVATE KEY-----", "-----END RSA PRIVATE KEY-----");
  if (key.length() > 0) {
    return key;
  }

  key = extractFirstPemBlock(rawText, "-----BEGIN EC PRIVATE KEY-----", "-----END EC PRIVATE KEY-----");
  return key;
}

String formatMbedtlsError(int code) {
  char errBuf[160] = {0};
  mbedtls_strerror(code, errBuf, sizeof(errBuf));
  return String(errBuf);
}

PemCheckResult validateCertPem(const String &pemText) {
  PemCheckResult result = {false, 0, ""};
  if (pemText.length() == 0) {
    result.code = -1;
    result.details = "No certificate PEM block found";
    return result;
  }

  mbedtls_x509_crt crt;
  mbedtls_x509_crt_init(&crt);
  int ret = mbedtls_x509_crt_parse(&crt,
                                   reinterpret_cast<const unsigned char*>(pemText.c_str()),
                                   pemText.length() + 1);
  if (ret == 0) {
    result.ok = true;
    result.details = "OK";
  } else {
    result.code = ret;
    result.details = formatMbedtlsError(ret);
  }
  mbedtls_x509_crt_free(&crt);
  return result;
}

PemCheckResult validatePrivateKeyPem(const String &pemText) {
  PemCheckResult result = {false, 0, ""};
  if (pemText.length() == 0) {
    result.code = -1;
    result.details = "No private key PEM block found";
    return result;
  }

  mbedtls_pk_context pk;
  mbedtls_pk_init(&pk);
  int ret = mbedtls_pk_parse_key(&pk,
                                 reinterpret_cast<const unsigned char*>(pemText.c_str()),
                                 pemText.length() + 1,
                                 nullptr,
                                 0);
  if (ret == 0) {
    result.ok = true;
    result.details = "OK";
  } else {
    result.code = ret;
    result.details = formatMbedtlsError(ret);
  }
  mbedtls_pk_free(&pk);
  return result;
}

PemCheckResult validateCertKeyPair(const String &certPem, const String &keyPem) {
  PemCheckResult result = {false, 0, ""};
  if (certPem.length() == 0 || keyPem.length() == 0) {
    result.code = -1;
    result.details = "Missing cert or key PEM";
    return result;
  }

  mbedtls_x509_crt crt;
  mbedtls_x509_crt_init(&crt);
  mbedtls_pk_context pk;
  mbedtls_pk_init(&pk);

  int certRet = mbedtls_x509_crt_parse(&crt,
                                       reinterpret_cast<const unsigned char*>(certPem.c_str()),
                                       certPem.length() + 1);
  if (certRet != 0) {
    result.code = certRet;
    result.details = "Cert parse failed: " + formatMbedtlsError(certRet);
    mbedtls_pk_free(&pk);
    mbedtls_x509_crt_free(&crt);
    return result;
  }

  int keyRet = mbedtls_pk_parse_key(&pk,
                                    reinterpret_cast<const unsigned char*>(keyPem.c_str()),
                                    keyPem.length() + 1,
                                    nullptr,
                                    0);
  if (keyRet != 0) {
    result.code = keyRet;
    result.details = "Key parse failed: " + formatMbedtlsError(keyRet);
    mbedtls_pk_free(&pk);
    mbedtls_x509_crt_free(&crt);
    return result;
  }

  int pairRet = mbedtls_pk_check_pair(&crt.pk, &pk);
  if (pairRet == 0) {
    result.ok = true;
    result.details = "OK";
  } else {
    result.code = pairRet;
    result.details = formatMbedtlsError(pairRet);
  }

  mbedtls_pk_free(&pk);
  mbedtls_x509_crt_free(&crt);
  return result;
}

String bytesToHex(const unsigned char *bytes, size_t len) {
  String out;
  out.reserve(len * 2);
  const char *hex = "0123456789abcdef";
  for (size_t i = 0; i < len; i++) {
    unsigned char b = bytes[i];
    out += hex[(b >> 4) & 0x0F];
    out += hex[b & 0x0F];
  }
  return out;
}

ParsedCertInfo parseCertInfoFromPem(const String &pemText) {
  ParsedCertInfo info = {false, "", "", "", "", ""};
  if (pemText.length() == 0) {
    info.details = "No certificate PEM block found";
    return info;
  }

  mbedtls_x509_crt crt;
  mbedtls_x509_crt_init(&crt);
  int ret = mbedtls_x509_crt_parse(&crt,
                                   reinterpret_cast<const unsigned char*>(pemText.c_str()),
                                   pemText.length() + 1);
  if (ret != 0) {
    info.details = formatMbedtlsError(ret);
    mbedtls_x509_crt_free(&crt);
    return info;
  }

  char dnBuf[256] = {0};
  if (mbedtls_x509_dn_gets(dnBuf, sizeof(dnBuf), &crt.subject) > 0) {
    info.subject = String(dnBuf);
  }
  memset(dnBuf, 0, sizeof(dnBuf));
  if (mbedtls_x509_dn_gets(dnBuf, sizeof(dnBuf), &crt.issuer) > 0) {
    info.issuer = String(dnBuf);
  }

  char serialBuf[160] = {0};
  if (mbedtls_x509_serial_gets(serialBuf, sizeof(serialBuf), &crt.serial) > 0) {
    info.serial = String(serialBuf);
  }

  unsigned char hash[32] = {0};
  if (mbedtls_sha256_ret(crt.raw.p, crt.raw.len, hash, 0) == 0) {
    info.sha256 = bytesToHex(hash, sizeof(hash));
  }

  info.ok = true;
  info.details = "OK";
  mbedtls_x509_crt_free(&crt);
  return info;
}

ParsedCertInfo parseCertInfoFromPeer(const mbedtls_x509_crt *crt) {
  ParsedCertInfo info = {false, "", "", "", "", ""};
  if (crt == nullptr || crt->raw.p == nullptr || crt->raw.len == 0) {
    info.details = "Peer certificate not available";
    return info;
  }

  char dnBuf[256] = {0};
  if (mbedtls_x509_dn_gets(dnBuf, sizeof(dnBuf), &crt->subject) > 0) {
    info.subject = String(dnBuf);
  }
  memset(dnBuf, 0, sizeof(dnBuf));
  if (mbedtls_x509_dn_gets(dnBuf, sizeof(dnBuf), &crt->issuer) > 0) {
    info.issuer = String(dnBuf);
  }

  char serialBuf[160] = {0};
  if (mbedtls_x509_serial_gets(serialBuf, sizeof(serialBuf), &crt->serial) > 0) {
    info.serial = String(serialBuf);
  }

  unsigned char hash[32] = {0};
  if (mbedtls_sha256_ret(crt->raw.p, crt->raw.len, hash, 0) == 0) {
    info.sha256 = bytesToHex(hash, sizeof(hash));
  }

  info.ok = true;
  info.details = "OK";
  return info;
}

String sanitizeHostValue(const String &rawHost) {
  String host = rawHost;
  host.trim();

  int schemeIdx = host.indexOf("://");
  if (schemeIdx >= 0) {
    host = host.substring(schemeIdx + 3);
  }

  int slashIdx = host.indexOf('/');
  if (slashIdx >= 0) {
    host = host.substring(0, slashIdx);
  }

  int firstColon = host.indexOf(':');
  if (firstColon > 0 && host.indexOf(':', firstColon + 1) < 0) {
    host = host.substring(0, firstColon);
  }

  host.trim();
  host.toLowerCase();
  return host;
}

String sanitizeFsPathValue(const String &rawPath) {
  String path = rawPath;
  path.trim();

  if (!path.startsWith("/")) {
    path = "/" + path;
  }

  while (path.indexOf("//") >= 0) {
    path.replace("//", "/");
  }

  return path;
}

bool isFatalTlsError(int errCode) {
  // Fatal classes where immediate retry is unlikely to succeed.
  // -9186  ASN1 tag/value invalid (malformed cert/key input)
  // -28928 SSL bad input parameters
  // -32512 SSL alloc failed: retrying immediately usually worsens pressure
  return (errCode == -9186 || errCode == -28928 || errCode == -32512);
}

String xmlEscape(const String &raw) {
  String text = raw;
  text.replace("&", "&amp;");
  text.replace("\"", "&quot;");
  text.replace("'", "&apos;");
  text.replace("<", "&lt;");
  text.replace(">", "&gt;");
  return text;
}

String formatTakTimestamp(time_t value) {
  if (value <= 0) {
    return "1970-01-01T00:00:00Z";
  }

  struct tm *utc = gmtime(&value);
  if (utc == nullptr) {
    return "1970-01-01T00:00:00Z";
  }

  char buffer[32] = {0};
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", utc);
  return String(buffer);
}

String buildCotEventXml(const SensorData &sensorData) {
  time_t now = time(nullptr);
  if (now < 1577836800L) {
    return "";
  }

  String uid = strlen(config.takUID) > 0 ? String(config.takUID) : String(config.deviceName);
  String callsign = strlen(config.takCallsign) > 0 ? String(config.takCallsign) : String(config.deviceName);

  String nowIso = formatTakTimestamp(now);
  String staleIso = formatTakTimestamp(now + kTakCotStaleSeconds);

  String lat = String(sensorData.ownLat, 7);
  String lon = String(sensorData.ownLon, 7);
  String course = String(sensorData.compassHeading, 1);

  String cot = "";
  cot.reserve(512);
  cot += "<event version=\"2.0\" uid=\"" + xmlEscape(uid) + "\" type=\"a-f-G-U-C\" how=\"m-g\" time=\"" + nowIso + "\" start=\"" + nowIso + "\" stale=\"" + staleIso + "\">";
  cot += "<point lat=\"" + lat + "\" lon=\"" + lon + "\" hae=\"0.0\" ce=\"10.0\" le=\"10.0\"/>";
  cot += "<detail>";
  cot += "<contact callsign=\"" + xmlEscape(callsign) + "\"/>";
  cot += "<track course=\"" + course + "\" speed=\"0.0\"/>";
  cot += "</detail>";
  cot += "</event>\n";
  return cot;
}

void copyToBuffer(const String &value, char *buffer, size_t bufferSize) {
  value.toCharArray(buffer, bufferSize);
}

void ensureTakDirectories() {
  createDir(LittleFS, "/certs");
}

void refreshTakFileCache() {
  takCertExists = LittleFS.exists(config.takClientCertPath);
  takKeyExists = LittleFS.exists(config.takClientKeyPath);
  takCAExists = LittleFS.exists(config.takCACertPath);
  config.takConfigured = strlen(config.takServer) > 0 && config.takPort > 0;
}

void buildTakUidIfEmpty() {
  if (strlen(config.takUID) > 0) {
    return;
  }

  String mac = WiFi.macAddress();
  mac.replace(":", "");
  String uid = String(config.deviceName) + "-" + mac;
  copyToBuffer(uid, config.takUID, sizeof(config.takUID));
}

void normalizeTakConfig() {
  copyToBuffer(sanitizeHostValue(String(config.takServer)), config.takServer, sizeof(config.takServer));
  copyToBuffer(sanitizeHostValue(String(config.takTLSServerName)), config.takTLSServerName, sizeof(config.takTLSServerName));
  copyToBuffer(sanitizeFsPathValue(String(config.takCACertPath)), config.takCACertPath, sizeof(config.takCACertPath));
  copyToBuffer(sanitizeFsPathValue(String(config.takClientCertPath)), config.takClientCertPath, sizeof(config.takClientCertPath));
  copyToBuffer(sanitizeFsPathValue(String(config.takClientKeyPath)), config.takClientKeyPath, sizeof(config.takClientKeyPath));
  copyToBuffer(sanitizeFsPathValue(String(config.takClientP12Path)), config.takClientP12Path, sizeof(config.takClientP12Path));
  copyToBuffer(sanitizeFsPathValue(String(config.takTruststoreP12Path)), config.takTruststoreP12Path, sizeof(config.takTruststoreP12Path));

  // Sanitize reconnect values loaded from JSON at boot to avoid aggressive retry storms.
  if (config.takReconnectInitialDelayMs == 0) {
    config.takReconnectInitialDelayMs = 5000;
  }
  if (config.takReconnectMaxDelayMs < config.takReconnectInitialDelayMs) {
    config.takReconnectMaxDelayMs = config.takReconnectInitialDelayMs;
  }
  if (config.takReconnectBackoffMultiplier <= 1.0f) {
    config.takReconnectBackoffMultiplier = 1.5f;
  }

  buildTakUidIfEmpty();
  refreshTakFileCache();
}

bool loadClientCredentialsIfAvailable() {
  if (!config.takUseClientCert) {
    takLoadedClientCertPem = "";
    takLoadedClientKeyPem = "";
    return true;
  }
  if (!takCertExists || !takKeyExists) {
    takLastMessage = "Missing client certificate or key";
    return false;
  }

  takLoadedClientCertPem = extractFirstPemBlock(readFile(LittleFS, config.takClientCertPath),
                                                "-----BEGIN CERTIFICATE-----",
                                                "-----END CERTIFICATE-----");
  takLoadedClientKeyPem = extractPrivateKeyPemBlock(readFile(LittleFS, config.takClientKeyPath));
  PemCheckResult certCheck = validateCertPem(takLoadedClientCertPem);
  if (!certCheck.ok) {
    takLastMessage = "Client certificate PEM invalid: " + certCheck.details;
    return false;
  }
  PemCheckResult keyCheck = validatePrivateKeyPem(takLoadedClientKeyPem);
  if (!keyCheck.ok) {
    takLastMessage = "Client private key PEM invalid: " + keyCheck.details;
    return false;
  }

  takSecureClient.setCertificate(takLoadedClientCertPem.c_str());
  takSecureClient.setPrivateKey(takLoadedClientKeyPem.c_str());
  return true;
}

bool configureTakClient() {
  takClient = config.takSSL ? static_cast<WiFiClient *>(&takSecureClient)
                            : static_cast<WiFiClient *>(&takPlainClient);
  if (!config.takSSL) {
    return true;
  }

  takSecureClient.stop();
  takSecureClient.setHandshakeTimeout(15);

  // ESP32 WiFiClientSecure does not load client cert/key when insecure mode is enabled.
  // If client cert auth is requested, keep CA verification enabled so mTLS can function.
  bool requireVerifiedTlsForClientCert = (!config.takVerifyCert && config.takUseClientCert);
  if (requireVerifiedTlsForClientCert) {
    takLastMessage = "TAK note: forcing cert verification because client cert auth requires secure mode on ESP32";
  }

  if (config.takVerifyCert || requireVerifiedTlsForClientCert) {
    if (!takCAExists) {
      takLastMessage = "Missing CA certificate";
      return false;
    }

    takLoadedCaPem = extractAllCertificatePemBlocks(readFile(LittleFS, config.takCACertPath));
    PemCheckResult caCheck = validateCertPem(takLoadedCaPem);
    if (!caCheck.ok) {
      takLastMessage = "CA certificate PEM invalid: " + caCheck.details;
      return false;
    }
    takSecureClient.setCACert(takLoadedCaPem.c_str());
  } else {
    takLoadedCaPem = "";
    takSecureClient.setInsecure();
  }

  return loadClientCredentialsIfAvailable();
}

void persistTakConfig() {
  normalizeTakConfig();
  saveConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str());
}

void saveTakMetrics() {
  String path = jsonDir + "tak_metrics.json";
  LittleFS.remove(path.c_str());
  File file = LittleFS.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("TAK: failed to open metrics file for writing");
    return;
  }
  JsonDocument doc;
  doc["totalReconnectAttempts"] = takReconnectTotalAttempts;
  doc["totalReconnectSuccesses"] = takReconnectTotalSuccesses;
  serializeJson(doc, file);
  file.close();
}

void loadTakMetrics() {
  String path = jsonDir + "tak_metrics.json";
  File file = LittleFS.open(path.c_str(), FILE_READ);
  if (!file) {
    return;  // No metrics file yet; start from zero
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, file);
  file.close();
  if (err) {
    return;
  }
  takReconnectTotalAttempts = doc["totalReconnectAttempts"] | 0u;
  takReconnectTotalSuccesses = doc["totalReconnectSuccesses"] | 0u;
}

void resetReconnectState() {
  takLastDisconnectMs = 0;
  takReconnectAttempts = 0;
  takReconnectCurrentDelayMs = 0;
  takReconnectNextAttemptMs = 0;
  takReconnectLastReason = "";
  takReconnectGivenUp = false;
}

void noteTakDisconnected() {
  if (takLastDisconnectMs != 0) {
    return;  // Already tracking this disconnect event
  }
  unsigned long nowMs = millis();
  takLastDisconnectMs = nowMs;
  takReconnectAttempts = 0;
  takReconnectCurrentDelayMs = config.takReconnectInitialDelayMs;
  takReconnectNextAttemptMs = nowMs + takReconnectCurrentDelayMs;
  takReconnectGivenUp = false;
  takReconnectLastReason = "Connection lost";
}

bool shouldAttemptReconnect() {
  if (!config.takReconnectEnabled) return false;
  if (takConnected) return false;
  if (takLastDisconnectMs == 0) return false;  // Not in a reconnect cycle
  if (WiFi.status() != WL_CONNECTED) return false;
  if (takReconnectGivenUp) return false;

  if (takReconnectAttempts >= kTakReconnectMaxAttemptsPerCycle) {
    takReconnectGivenUp = true;
    takLastMessage = "TAK reconnect: giving up after " + String(kTakReconnectMaxAttemptsPerCycle) + " attempts";
    saveTakMetrics();
    return false;
  }

  unsigned long nowMs = millis();
  if (config.takReconnectMaxDurationMs > 0 &&
      (nowMs - takLastDisconnectMs) > config.takReconnectMaxDurationMs) {
    takReconnectGivenUp = true;
    takLastMessage = "TAK reconnect: giving up after " +
                     String(config.takReconnectMaxDurationMs / 60000) + " min";
    saveTakMetrics();
    return false;
  }

  return (nowMs >= takReconnectNextAttemptMs);
}

void attemptTakReconnect() {
  takReconnectTotalAttempts++;
  String msg;
  bool result = connectTAK(msg);
  if (result) {
    takReconnectTotalSuccesses++;
    resetReconnectState();
    saveTakMetrics();
  } else {
    takReconnectAttempts++;
    takReconnectLastReason = msg;
    // Advance backoff: multiply current delay, cap at max
    unsigned long nextDelay = (unsigned long)((float)takReconnectCurrentDelayMs *
                                              config.takReconnectBackoffMultiplier);
    if (nextDelay > config.takReconnectMaxDelayMs || nextDelay <= takReconnectCurrentDelayMs) {
      nextDelay = config.takReconnectMaxDelayMs;
    }
    takReconnectCurrentDelayMs = nextDelay;
    takReconnectNextAttemptMs = millis() + takReconnectCurrentDelayMs;
    saveTakMetrics();
  }
}


bool storePemArtifact(const char *path, const String &content) {
  if (content.length() == 0) {
    return false;
  }
  deleteFile(LittleFS, path);
  return writeStringToFile(LittleFS, path, content);
}

}  // namespace

void setupTAK() {
  ensureTakDirectories();
  normalizeTakConfig();
  loadTakMetrics();
}

void getTAKConfig(JsonDocument &doc) {
  normalizeTakConfig();
  doc["takEnabled"] = config.takEnabled;
  doc["takSSL"] = config.takSSL;
  doc["takVerifyCert"] = config.takVerifyCert;
  doc["takUseClientCert"] = config.takUseClientCert;
  doc["takConfigured"] = config.takConfigured;
  doc["takPackageImported"] = config.takPackageImported;
  doc["takServer"] = config.takServer;
  doc["takTLSServerName"] = config.takTLSServerName;
  doc["takPort"] = config.takPort;
  doc["takCallsign"] = config.takCallsign;
  doc["takUID"] = config.takUID;
  doc["takDescription"] = config.takDescription;
  doc["takCACertPath"] = config.takCACertPath;
  doc["takClientCertPath"] = config.takClientCertPath;
  doc["takClientKeyPath"] = config.takClientKeyPath;
  doc["takClientP12Path"] = config.takClientP12Path;
  doc["takTruststoreP12Path"] = config.takTruststoreP12Path;
  doc["takReconnectEnabled"] = config.takReconnectEnabled;
  doc["takReconnectOnWifiReconnect"] = config.takReconnectOnWifiReconnect;
  doc["takReconnectInitialDelayMs"] = config.takReconnectInitialDelayMs;
  doc["takReconnectMaxDelayMs"] = config.takReconnectMaxDelayMs;
  doc["takReconnectBackoffMultiplier"] = config.takReconnectBackoffMultiplier;
  doc["takReconnectMaxDurationMs"] = config.takReconnectMaxDurationMs;
}

void getTAKStatus(JsonDocument &doc) {
  refreshTakFileCache();
  takConnected = takClient->connected();
  bool manualConnecting = takManualConnectRequested || takManualConnectInProgress;
  doc["connected"] = takConnected;
  doc["connecting"] = (!takConnected && manualConnecting);
  doc["enabled"] = config.takEnabled;
  doc["configured"] = config.takConfigured;
  doc["packageImported"] = config.takPackageImported;
  doc["certExists"] = takCertExists;
  doc["keyExists"] = takKeyExists;
  doc["caExists"] = takCAExists;
  doc["ssl"] = config.takSSL;
  doc["server"] = config.takServer;
  doc["port"] = config.takPort;
  doc["uid"] = config.takUID;
  doc["message"] = takLastMessage;
  doc["lastConnectAttemptMs"] = takLastConnectAttemptMs;
  doc["lastCotAttemptMs"] = takLastCotAttemptMs;
  doc["lastCotSentMs"] = takLastCotSentMs;
  doc["cotTxCount"] = takCotTxCount;
  doc["cotTxFailCount"] = takCotTxFailCount;
  // Reconnection metrics
  unsigned long nowMs = millis();
  doc["reconnectEnabled"] = config.takReconnectEnabled;
  doc["reconnecting"] = (takLastDisconnectMs > 0 && !takConnected && !takReconnectGivenUp);
  doc["reconnectGivenUp"] = takReconnectGivenUp;
  doc["reconnectAttempts"] = takReconnectAttempts;
  doc["reconnectTotalAttempts"] = takReconnectTotalAttempts;
  doc["reconnectTotalSuccesses"] = takReconnectTotalSuccesses;
  doc["reconnectLastReason"] = takReconnectLastReason;
  doc["timeSinceDisconnectMs"] = takLastDisconnectMs > 0 ? (nowMs - takLastDisconnectMs) : 0;
  doc["nextReconnectInMs"] = (takReconnectNextAttemptMs > nowMs && !takConnected)
                               ? (takReconnectNextAttemptMs - nowMs) : 0;
}

void getTAKCertDiagnostics(JsonDocument &doc) {
  String caRaw = readFile(LittleFS, config.takCACertPath);
  String certRaw = readFile(LittleFS, config.takClientCertPath);
  String keyRaw = readFile(LittleFS, config.takClientKeyPath);

  String caPem = extractAllCertificatePemBlocks(caRaw);
  String certPem = extractFirstPemBlock(certRaw,
                                        "-----BEGIN CERTIFICATE-----",
                                        "-----END CERTIFICATE-----");
  String keyPem = extractPrivateKeyPemBlock(keyRaw);

  PemCheckResult caCheck = validateCertPem(caPem);
  PemCheckResult certCheck = validateCertPem(certPem);
  PemCheckResult keyCheck = validatePrivateKeyPem(keyPem);
  PemCheckResult pairCheck = validateCertKeyPair(certPem, keyPem);
  ParsedCertInfo importedCAInfo = parseCertInfoFromPem(caPem);
  ParsedCertInfo importedClientInfo = parseCertInfoFromPem(certPem);

  doc["caPath"] = config.takCACertPath;
  doc["clientCertPath"] = config.takClientCertPath;
  doc["clientKeyPath"] = config.takClientKeyPath;

  doc["caRawLength"] = caRaw.length();
  doc["clientCertRawLength"] = certRaw.length();
  doc["clientKeyRawLength"] = keyRaw.length();

  doc["caPemLength"] = caPem.length();
  doc["clientCertPemLength"] = certPem.length();
  doc["clientKeyPemLength"] = keyPem.length();

  doc["caPemValid"] = caCheck.ok;
  doc["caPemCode"] = caCheck.code;
  doc["caPemDetails"] = caCheck.details;

  doc["clientCertPemValid"] = certCheck.ok;
  doc["clientCertPemCode"] = certCheck.code;
  doc["clientCertPemDetails"] = certCheck.details;

  doc["clientKeyPemValid"] = keyCheck.ok;
  doc["clientKeyPemCode"] = keyCheck.code;
  doc["clientKeyPemDetails"] = keyCheck.details;

  doc["clientCertKeyPairValid"] = pairCheck.ok;
  doc["clientCertKeyPairCode"] = pairCheck.code;
  doc["clientCertKeyPairDetails"] = pairCheck.details;

  doc["importedCASubject"] = importedCAInfo.subject;
  doc["importedCAIssuer"] = importedCAInfo.issuer;
  doc["importedCASerial"] = importedCAInfo.serial;
  doc["importedCASha256"] = importedCAInfo.sha256;
  doc["importedCAParseDetails"] = importedCAInfo.details;

  doc["importedClientCertSubject"] = importedClientInfo.subject;
  doc["importedClientCertIssuer"] = importedClientInfo.issuer;
  doc["importedClientCertSerial"] = importedClientInfo.serial;
  doc["importedClientCertSha256"] = importedClientInfo.sha256;
  doc["importedClientCertParseDetails"] = importedClientInfo.details;

  time_t nowTs = time(nullptr);
  doc["systemEpoch"] = (long long)nowTs;
  doc["systemTimeSynced"] = (nowTs > 1577836800L);
  char timeBufUtc[32] = {};
  struct tm *tmUtc = gmtime(&nowTs);
  if (tmUtc) {
    strftime(timeBufUtc, sizeof(timeBufUtc), "%Y-%m-%dT%H:%M:%SZ", tmUtc);
  }
  doc["systemTimeUtc"] = timeBufUtc;
  char timeBufLocal[32] = {};
  struct tm *tmLocal = localtime(&nowTs);
  if (tmLocal) {
    strftime(timeBufLocal, sizeof(timeBufLocal), "%Y-%m-%dT%H:%M:%S", tmLocal);
  }
  doc["systemTimeLocal"] = timeBufLocal;

  doc["takServer"] = config.takServer;
  doc["takTLSServerName"] = config.takTLSServerName;
  doc["takPort"] = config.takPort;
  doc["takSSL"] = config.takSSL;
  doc["takVerifyCert"] = config.takVerifyCert;
  doc["takUseClientCert"] = config.takUseClientCert;

  bool probeConnected = false;
  String probeMessage = "Skipped";
  ParsedCertInfo peerInfo = {false, "", "", "", "", ""};
  if (WiFi.status() == WL_CONNECTED && strlen(config.takServer) > 0 && config.takPort > 0) {
    const char *tlsServerName = strlen(config.takTLSServerName) > 0 ? config.takTLSServerName : config.takServer;
    const char *probeClientCert = certPem.length() > 0 ? certPem.c_str() : nullptr;
    const char *probeClientKey = keyPem.length() > 0 ? keyPem.c_str() : nullptr;

    IPAddress probeIp;
    if (!WiFi.hostByName(config.takServer, probeIp)) {
      probeMessage = "DNS resolution failed for TAK server";
      doc["serverProbeConnected"] = false;
      doc["serverProbeMessage"] = probeMessage;
      doc["serverCertSubject"] = peerInfo.subject;
      doc["serverCertIssuer"] = peerInfo.issuer;
      doc["serverCertSerial"] = peerInfo.serial;
      doc["serverCertSha256"] = peerInfo.sha256;
      doc["serverIssuerMatchesImportedCA"] = false;
      return;
    }

    doc["serverProbeIp"] = probeIp.toString();

    auto runProbe = [&](const char *label, const char *sniHost, bool useClientCert) {
      WiFiClientSecure c;
      c.setHandshakeTimeout(10);
      const char *certArg = useClientCert ? probeClientCert : nullptr;
      const char *keyArg = useClientCert ? probeClientKey : nullptr;
      const char *probeCa = caPem.length() > 0 ? caPem.c_str() : nullptr;
      bool ok = c.connect(probeIp, config.takPort, sniHost, probeCa, certArg, keyArg);

      JsonDocument probeDoc;
      probeDoc["label"] = label;
      probeDoc["sniHost"] = sniHost ? sniHost : "";
      probeDoc["useClientCert"] = useClientCert;
      probeDoc["connected"] = ok;
      if (!ok) {
        char ebuf[192] = {0};
        int ecode = c.lastError(ebuf, sizeof(ebuf));
        probeDoc["errorCode"] = ecode;
        probeDoc["error"] = String(ebuf);
      }

      String out;
      serializeJson(probeDoc, out);
      doc[label] = out;
      c.stop();
      return ok;
    };

    bool probeNoCertServerName = runProbe("probeNoCertServerName", config.takServer, false);
    bool probeNoCertTlsName = runProbe("probeNoCertTlsName", tlsServerName, false);
    bool probeWithCertServerName = runProbe("probeWithCertServerName", config.takServer, true);
    bool probeWithCertTlsName = runProbe("probeWithCertTlsName", tlsServerName, true);

    WiFiClientSecure probeClient;
    probeClient.setHandshakeTimeout(10);
    const char *probeCa = caPem.length() > 0 ? caPem.c_str() : nullptr;
    probeConnected = probeClient.connect(probeIp, config.takPort, tlsServerName, probeCa, probeClientCert, probeClientKey);
    if (probeConnected) {
      const mbedtls_x509_crt *peer = probeClient.getPeerCertificate();
      peerInfo = parseCertInfoFromPeer(peer);
      probeMessage = peerInfo.ok ? "OK" : peerInfo.details;
    } else {
      char errBuf[192] = {0};
      int errCode = probeClient.lastError(errBuf, sizeof(errBuf));
      probeMessage = "TLS probe failed (" + String(errCode) + "): " + String(errBuf);
    }
    doc["probeSummary"] = String("noCert/server=") + (probeNoCertServerName ? "ok" : "fail") +
                          ", noCert/tlsName=" + (probeNoCertTlsName ? "ok" : "fail") +
                          ", cert/server=" + (probeWithCertServerName ? "ok" : "fail") +
                          ", cert/tlsName=" + (probeWithCertTlsName ? "ok" : "fail");
    probeClient.stop();
  }

  doc["serverProbeConnected"] = probeConnected;
  doc["serverProbeMessage"] = probeMessage;
  doc["serverCertSubject"] = peerInfo.subject;
  doc["serverCertIssuer"] = peerInfo.issuer;
  doc["serverCertSerial"] = peerInfo.serial;
  doc["serverCertSha256"] = peerInfo.sha256;

  bool issuerMatchesImportedCA = (peerInfo.issuer.length() > 0 && importedCAInfo.subject.length() > 0 && peerInfo.issuer == importedCAInfo.subject);
  doc["serverIssuerMatchesImportedCA"] = issuerMatchesImportedCA;
}

bool requestTAKConnect(String &message) {
  normalizeTakConfig();

  if (!config.takEnabled) {
    message = "TAK is disabled";
    takLastMessage = message;
    return false;
  }
  if (!config.takConfigured) {
    message = "TAK server or port is not configured";
    takLastMessage = message;
    return false;
  }
  if (WiFi.status() != WL_CONNECTED) {
    message = "WiFi is not connected";
    takLastMessage = message;
    return false;
  }
  if (takConnected) {
    message = "TAK connection already open";
    takLastMessage = message;
    return true;
  }

  takManualConnectRequested = true;
  takManualConnectInProgress = false;
  takReconnectGivenUp = false;
  takLastMessage = "TAK connect requested";
  message = takLastMessage;
  return true;
}

bool updateTAKConfigFromJson(const JsonDocument &doc, String &message) {
  JsonVariantConst obj = doc.as<JsonVariantConst>();

  config.takEnabled = obj["takEnabled"] | config.takEnabled;
  config.takSSL = obj["takSSL"] | config.takSSL;
  config.takVerifyCert = obj["takVerifyCert"] | config.takVerifyCert;
  config.takUseClientCert = obj["takUseClientCert"] | config.takUseClientCert;
  config.takPort = obj["takPort"] | config.takPort;

  copyToBuffer(obj["takServer"] | String(config.takServer), config.takServer, sizeof(config.takServer));
  copyToBuffer(obj["takTLSServerName"] | String(config.takTLSServerName), config.takTLSServerName, sizeof(config.takTLSServerName));
  copyToBuffer(obj["takCallsign"] | String(config.takCallsign), config.takCallsign, sizeof(config.takCallsign));
  copyToBuffer(obj["takUID"] | String(config.takUID), config.takUID, sizeof(config.takUID));
  copyToBuffer(obj["takDescription"] | String(config.takDescription), config.takDescription, sizeof(config.takDescription));
  copyToBuffer(obj["takCACertPath"] | String(config.takCACertPath), config.takCACertPath, sizeof(config.takCACertPath));
  copyToBuffer(obj["takClientCertPath"] | String(config.takClientCertPath), config.takClientCertPath, sizeof(config.takClientCertPath));
  copyToBuffer(obj["takClientKeyPath"] | String(config.takClientKeyPath), config.takClientKeyPath, sizeof(config.takClientKeyPath));
  copyToBuffer(obj["takClientP12Path"] | String(config.takClientP12Path), config.takClientP12Path, sizeof(config.takClientP12Path));
  copyToBuffer(obj["takTruststoreP12Path"] | String(config.takTruststoreP12Path), config.takTruststoreP12Path, sizeof(config.takTruststoreP12Path));

  config.takReconnectEnabled = obj["takReconnectEnabled"] | config.takReconnectEnabled;
  config.takReconnectOnWifiReconnect = obj["takReconnectOnWifiReconnect"] | config.takReconnectOnWifiReconnect;
  {
    uint32_t v = obj["takReconnectInitialDelayMs"] | config.takReconnectInitialDelayMs;
    config.takReconnectInitialDelayMs = v > 0 ? v : 1000u;
  }
  {
    uint32_t v = obj["takReconnectMaxDelayMs"] | config.takReconnectMaxDelayMs;
    config.takReconnectMaxDelayMs = (v >= config.takReconnectInitialDelayMs) ? v : config.takReconnectInitialDelayMs;
  }
  {
    float v = obj["takReconnectBackoffMultiplier"] | config.takReconnectBackoffMultiplier;
    config.takReconnectBackoffMultiplier = v > 1.0f ? v : 1.1f;
  }
  config.takReconnectMaxDurationMs = obj["takReconnectMaxDurationMs"] | config.takReconnectMaxDurationMs;

  persistTakConfig();
  message = "TAK settings saved";
  takLastMessage = message;
  return true;
}

bool importTAKPackageData(const JsonDocument &doc, String &message) {
  String takServer = doc["takServer"] | "";
  uint16_t takPort = doc["takPort"] | 8089;
  bool takSSL = doc["takSSL"] | true;
  String description = doc["takDescription"] | "";
  description.trim();

  int underscoreIndex = description.indexOf('_');
  String takTLSServerName = (underscoreIndex > 0)
  ? description.substring(0, underscoreIndex)
  : description;
  takTLSServerName.trim();
  String clientCertPem = extractFirstPemBlock(doc["clientCertPem"] | "",
                                               "-----BEGIN CERTIFICATE-----",
                                               "-----END CERTIFICATE-----");
  String clientKeyPem = extractPrivateKeyPemBlock(doc["clientKeyPem"] | "");
  String caCertPem = extractAllCertificatePemBlocks(doc["caCertPem"] | "");

  if (takServer.length() == 0) {
    message = "Imported package did not contain a TAK server host";
    return false;
  }
  if (clientCertPem.length() == 0 || clientKeyPem.length() == 0 || caCertPem.length() == 0) {
    message = "Imported package PEM is invalid (cert/key/ca)";
    return false;
  }

  ensureTakDirectories();

  if (!storePemArtifact(config.takClientCertPath, clientCertPem) ||
      !storePemArtifact(config.takClientKeyPath, clientKeyPem) ||
      !storePemArtifact(config.takCACertPath, caCertPem)) {
    message = "Failed to store imported certificate files";
    return false;
  }

  config.takSSL = takSSL;
  config.takEnabled = doc["takEnabled"] | true;
  config.takPackageImported = true;
  copyToBuffer(takServer, config.takServer, sizeof(config.takServer));
  copyToBuffer(takTLSServerName, config.takTLSServerName, sizeof(config.takTLSServerName));
  config.takPort = takPort;
  copyToBuffer(description, config.takDescription, sizeof(config.takDescription));
  if (doc["takCallsign"].is<const char*>()) {
    copyToBuffer(doc["takCallsign"].as<String>(), config.takCallsign, sizeof(config.takCallsign));
  }
  if (doc["takUID"].is<const char*>()) {
    copyToBuffer(doc["takUID"].as<String>(), config.takUID, sizeof(config.takUID));
  }

  persistTakConfig();
  message = "TAK package data imported";
  takLastMessage = message;
  return true;
}

bool connectTAK(String &message) {
  takLastConnectAttemptMs = millis();
  normalizeTakConfig();

  if (!config.takEnabled) {
    message = "TAK is disabled";
    takLastMessage = message;
    return false;
  }
  if (!config.takConfigured) {
    message = "TAK server or port is not configured";
    takLastMessage = message;
    return false;
  }
  if (WiFi.status() != WL_CONNECTED) {
    message = "WiFi is not connected";
    takLastMessage = message;
    return false;
  }
  if (!configureTakClient()) {
    message = takLastMessage;
    return false;
  }

  if (takClient->connected()) {
    message = "TAK connection already open";
    takLastMessage = message;
    takConnected = true;
    return true;
  }

  // mbedTLS cert chain validation requires a valid system clock.
  // If NTP hasn't synced yet, fall back to GPS time when a fix is available.
  if (config.takSSL && config.takVerifyCert) {
    time_t now = time(nullptr);
    if (now < 1577836800L) {  // < Jan 1 2020 = definitely not synced
      if (gps.date.isValid() && gps.time.isValid() && gps.date.year() > 2000) {
        struct tm t = {};
        t.tm_year  = gps.date.year() - 1900;
        t.tm_mon   = gps.date.month() - 1;
        t.tm_mday  = gps.date.day();
        t.tm_hour  = gps.time.hour();
        t.tm_min   = gps.time.minute();
        t.tm_sec   = gps.time.second();
        t.tm_isdst = 0;
        time_t epochUtc = mktime(&t);
        struct timeval tv = { epochUtc, 0 };
        settimeofday(&tv, nullptr);
        Serial.printf("TAK: set system time from GPS: %s", ctime(&epochUtc));
      } else {
        message = "TAK TLS blocked: system clock not synced and no GPS time available";
        takLastMessage = message;
        takConnected = false;
        return false;
      }
    }
  }

  bool connected = false;
  if (config.takSSL) {
    IPAddress serverIp;
    if (!WiFi.hostByName(config.takServer, serverIp)) {
      message = "Failed to resolve TAK server host";
      takLastMessage = message;
      return false;
    }

    const char *tlsServerName = strlen(config.takTLSServerName) > 0 ? config.takTLSServerName : config.takServer;
    const char *caPem = (config.takVerifyCert && takLoadedCaPem.length() > 0) ? takLoadedCaPem.c_str() : nullptr;
    const char *clientCert = (config.takUseClientCert && takLoadedClientCertPem.length() > 0) ? takLoadedClientCertPem.c_str() : nullptr;
    const char *clientKey = (config.takUseClientCert && takLoadedClientKeyPem.length() > 0) ? takLoadedClientKeyPem.c_str() : nullptr;

    constexpr int maxAttempts = 3;
    constexpr int minAttemptsBeforeFail = 3;
    constexpr unsigned long retryDelayMs = 3000;
    int lastErrCode = 0;
    char lastErrBuf[192] = {0};
    String attemptLog = "";
    bool fatalError = false;

    for (int attempt = 1; attempt <= maxAttempts; attempt++) {
      const char *sniForAttempt = tlsServerName;

      // Do NOT call stop() here before connect() – configureTakClient() already stopped the
      // client, and after each failed connect() the WiFiClientSecure has internally freed its
      // ssl_client.  Calling stop() a second time before the next connect() invokes
      // WiFiClient::stop() on an already-closed socket, which corrupts the heap allocator's
      // block-list sentinel (producing "Bad head ... Expected 0xabba1234").
      connected = takSecureClient.connect(serverIp, config.takPort, sniForAttempt, caPem, clientCert, clientKey);
      if (connected) {
        if (attempt > 1) {
          message = "TAK connection established on attempt " + String(attempt) +
                    " (SNI=" + String(sniForAttempt) + ")";
          takLastMessage = message;
        }
        break;
      }

      // Capture the error before stop() potentially clears it.
      memset(lastErrBuf, 0, sizeof(lastErrBuf));
      lastErrCode = takSecureClient.lastError(lastErrBuf, sizeof(lastErrBuf));
      if (attemptLog.length() > 0) {
        attemptLog += "; ";
      }
      attemptLog += "#" + String(attempt) + "=" + String(lastErrCode) +
                    "[sni=" + String(sniForAttempt) + "]";

      fatalError = isFatalTlsError(lastErrCode);
      if (fatalError) {
        attemptLog += "(fatal)";
      }

      // Always clean up the failed TLS context before deciding whether to retry.
      takSecureClient.stop();

      if (fatalError && attempt >= minAttemptsBeforeFail) {
        break;
      }

      // Clean up after the failed attempt, then wait before retrying.
      if (attempt < maxAttempts) {
        delay(retryDelayMs);
      }
    }

    if (!connected && attemptLog.length() > 0) {
      message = "TAK TLS connect failed (" + String(lastErrCode) + "): " + String(lastErrBuf) + " | attempts: " + attemptLog;
      takLastMessage = message;
      takConnected = false;
      return false;
    }
  } else {
    connected = takClient->connect(config.takServer, config.takPort);
  }
  takConnected = connected;
  if (connected) {
    if (message.length() == 0) {
      message = "TAK connection established";
    }
  } else if (config.takSSL) {
    char errBuf[192] = {0};
    int errCode = takSecureClient.lastError(errBuf, sizeof(errBuf));
    message = "TAK TLS connect failed (" + String(errCode) + "): " + String(errBuf);
  } else {
    message = "TAK connection failed";
  }
  takLastMessage = message;
  return connected;
}

void disconnectTAK(const String &reason) {
  takPlainClient.stop();
  takSecureClient.stop();
  takConnected = false;
  takManualConnectRequested = false;
  takManualConnectInProgress = false;
  takLastMessage = reason.length() > 0 ? reason : "TAK disconnected";
  resetReconnectState();  // Manual disconnect: suppress auto-reconnect
}

void serviceTAK(const SensorData &sensorData) {
  if (!config.takEnabled || !config.takConfigured) {
    return;
  }

  unsigned long nowMs = millis();

  // Detect WiFi state transitions
  wl_status_t currentWifiStatus = (wl_status_t)WiFi.status();
  if (config.takReconnectOnWifiReconnect &&
      currentWifiStatus == WL_CONNECTED && takPrevWifiStatus != WL_CONNECTED &&
      !takConnected && takLastDisconnectMs > 0 && !takReconnectGivenUp) {
    // WiFi just came back up; schedule an immediate TAK reconnection attempt
    takReconnectNextAttemptMs = nowMs;
  }
  takPrevWifiStatus = currentWifiStatus;

  // Detect unexpected connection drop
  if (takConnected && !takClient->connected()) {
    takConnected = false;
    noteTakDisconnected();
  }

  if (!takConnected && takManualConnectRequested) {
    takManualConnectRequested = false;
    takManualConnectInProgress = true;
    takLastMessage = "TAK connecting...";
    attemptTakReconnect();
    takManualConnectInProgress = false;
    return;
  }

  // Attempt reconnection when disconnected
  if (!takConnected) {
    if (shouldAttemptReconnect()) {
      attemptTakReconnect();
    }
    return;
  }

  // CoT transmission
  if ((nowMs - takLastCotAttemptMs) < kTakCotIntervalMs) {
    return;
  }
  takLastCotAttemptMs = nowMs;

  if (sensorData.nrOfSatellites <= 0) {
    takCotTxFailCount++;
    takLastMessage = "TAK CoT skipped: no GPS fix";
    return;
  }

  if (!isfinite(sensorData.ownLat) || !isfinite(sensorData.ownLon)) {
    takCotTxFailCount++;
    takLastMessage = "TAK CoT skipped: invalid GPS coordinates";
    return;
  }

  String cot = buildCotEventXml(sensorData);
  if (cot.length() == 0) {
    takCotTxFailCount++;
    takLastMessage = "TAK CoT skipped: system time not synced";
    return;
  }

  size_t written = takClient->print(cot);
  if (written == cot.length()) {
    takCotTxCount++;
    takLastCotSentMs = nowMs;
    takLastMessage = "TAK CoT sent";
  } else {
    takCotTxFailCount++;
    takLastMessage = "TAK CoT send failed";
    if (!takClient->connected()) {
      takConnected = false;
      noteTakDisconnected();
    }
  }
}