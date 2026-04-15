#include "takhandler.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <LittleFS.h>
#include <HTTPClient.h>
#include <time.h>
#include <sys/time.h>
#include <mbedtls/pk.h>
#include <mbedtls/ecp.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>
#include <mbedtls/x509_csr.h>
#include <cctype>

#include "gps.h"
#include "filehandler.h"

extern Config config;
extern SensorData sensorData;
extern String jsonDir;
extern String fileConfigJSON;

bool takConnectRequested = false;
bool takDisconnectRequested = false;

static WiFiClient takPlainClient;
static WiFiClientSecure takSecureClient;
static WiFiClient* takClient = &takSecureClient;
static WiFiUDP takUdpClient;

static bool takClientUsesSSL = true;
static unsigned long lastTAKSendMs = 0;
static unsigned long lastReconnectAttemptMs = 0;
static const unsigned long TAK_RECONNECT_BACKOFF_MS = 10000;
static unsigned long lastEnrollAttemptMs = 0;
static bool takCertLoaded = false;
static String lastEnrollMessage = "not-started";

// Cached LittleFS existence state — updated on enroll/re-enroll and load.
// Avoids calling LittleFS.exists() on every UI poll from getTAKEnrollmentStatus.
static bool cachedCertExists = false;
static bool cachedKeyExists  = false;
static bool cachedP12Exists  = false;

static void refreshCertFileCache() {
  cachedCertExists = LittleFS.exists(config.takClientCertPath);
  cachedKeyExists  = LittleFS.exists(config.takClientKeyPath);
  cachedP12Exists  = LittleFS.exists(config.takClientP12Path);
}

static const char ISRG_ROOT_X1_PEM[] PROGMEM =
"-----BEGIN CERTIFICATE-----\n"
"MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n"
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n"
"WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n"
"ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n"
"MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n"
"h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n"
"0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n"
"A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n"
"T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n"
"B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n"
"B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n"
"KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n"
"OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n"
"jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n"
"qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n"
"rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n"
"HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n"
"hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n"
"ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n"
"3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n"
"NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n"
"ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n"
"TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n"
"jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n"
"oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n"
"4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n"
"mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n"
"emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n"
"-----END CERTIFICATE-----\n";

static bool ensureCertDir() {
  if (LittleFS.exists("/certs")) {
    return true;
  }
  return LittleFS.mkdir("/certs");
}

static bool isSystemTimeValidForTLS() {
  time_t now = time(nullptr);
  // Rough sanity threshold: 2023-11-14 UTC
  return now >= 1700000000;
}

static bool syncSystemTimeFromGPS() {
  String gpsDate = getGPSDate();
  int firstDash = gpsDate.indexOf('-');
  int secondDash = gpsDate.lastIndexOf('-');
  if (firstDash <= 0 || secondDash <= firstDash) {
    return false;
  }

  int year = gpsDate.substring(0, firstDash).toInt();
  int month = gpsDate.substring(firstDash + 1, secondDash).toInt();
  int day = gpsDate.substring(secondDash + 1).toInt();

  int rawTime = sensorData.gpsTime;
  int hours = 0;
  int mins = 0;
  int secs = 0;
  if (rawTime > 999999) {
    hours = (rawTime / 1000000) % 100;
    mins = (rawTime / 10000) % 100;
    secs = (rawTime / 100) % 100;
  } else {
    hours = (rawTime / 10000) % 100;
    mins = (rawTime / 100) % 100;
    secs = rawTime % 100;
  }

  if (year < 2024 || month < 1 || month > 12 || day < 1 || day > 31 ||
      hours < 0 || hours > 23 || mins < 0 || mins > 59 || secs < 0 || secs > 59) {
    return false;
  }

  struct tm tmUtc = {};
  tmUtc.tm_year = year - 1900;
  tmUtc.tm_mon = month - 1;
  tmUtc.tm_mday = day;
  tmUtc.tm_hour = hours;
  tmUtc.tm_min = mins;
  tmUtc.tm_sec = secs;
  tmUtc.tm_isdst = 0;

  setenv("TZ", "UTC0", 1);
  tzset();
  time_t epoch = mktime(&tmUtc);
  if (epoch <= 0) {
    return false;
  }

  struct timeval tv = {.tv_sec = epoch, .tv_usec = 0};
  if (settimeofday(&tv, nullptr) != 0) {
    return false;
  }

  Serial.printf("TAK: System time synced from GPS: %sT%02d:%02d:%02dZ\n", gpsDate.c_str(), hours, mins, secs);
  return true;
}

static bool ensureDirectoryPath(const String& dirPath) {
  if (dirPath.length() == 0 || dirPath == "/") {
    return true;
  }

  int start = 0;
  if (dirPath.charAt(0) == '/') {
    start = 1;
  }

  String current = "";
  for (int i = start; i < dirPath.length(); ++i) {
    if (dirPath.charAt(i) == '/') {
      continue;
    }

    int nextSlash = dirPath.indexOf('/', i);
    String segment = (nextSlash >= 0) ? dirPath.substring(i, nextSlash) : dirPath.substring(i);
    if (segment.length() > 0) {
      current += "/" + segment;
      if (!LittleFS.exists(current.c_str()) && !LittleFS.mkdir(current.c_str())) {
        return false;
      }
    }

    if (nextSlash < 0) {
      break;
    }
    i = nextSlash;
  }

  return true;
}

static String getParentDirectory(const char* path) {
  if (path == nullptr) {
    return "/";
  }

  String fullPath(path);
  int slashIndex = fullPath.lastIndexOf('/');
  if (slashIndex <= 0) {
    return "/";
  }
  return fullPath.substring(0, slashIndex);
}

static String readFileAsString(const char* path) {
  if (path == nullptr || strlen(path) == 0) {
    return String();
  }

  if (!LittleFS.exists(path)) {
    return String();
  }

  File file = LittleFS.open(path, FILE_READ);
  if (!file || file.isDirectory()) {
    return String();
  }

  String content;
  while (file.available()) {
    content += static_cast<char>(file.read());
  }
  file.close();
  return content;
}

static bool writeStringToFile(const char* path, const String& content) {
  String parentDir = getParentDirectory(path);
  if (!ensureDirectoryPath(parentDir)) {
    Serial.println("TAK: Failed to create parent directory for cert file.");
    return false;
  }

  // Keep the legacy behavior for /certs root as a fallback.
  ensureCertDir();

  File file = LittleFS.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("TAK: Failed to open cert file for writing.");
    return false;
  }

  size_t written = file.print(content);
  file.close();
  return written == content.length();
}

static String wrapBase64AsPemCertificate(const String& rawBase64) {
  String compact;
  compact.reserve(rawBase64.length());
  for (size_t i = 0; i < rawBase64.length(); ++i) {
    char c = rawBase64.charAt(i);
    if (!isspace(static_cast<unsigned char>(c))) {
      compact += c;
    }
  }

  if (compact.length() == 0) {
    return String();
  }

  String pem = "-----BEGIN CERTIFICATE-----\n";
  for (size_t i = 0; i < compact.length(); i += 64) {
    pem += compact.substring(i, i + 64);
    pem += "\n";
  }
  pem += "-----END CERTIFICATE-----\n";
  return pem;
}

static String normalizeCertificateFromApiField(const String& rawValue) {
  String value = rawValue;
  value.trim();
  if (value.length() == 0) {
    return String();
  }

  value.replace("\\n", "\n");
  if (value.indexOf("-----BEGIN CERTIFICATE-----") >= 0) {
    if (!value.endsWith("\n")) {
      value += "\n";
    }
    return value;
  }

  return wrapBase64AsPemCertificate(value);
}

// Always uses ISRG Root X1 for token HTTPS API calls because
// those endpoints sit behind a trusted public certificate, which
// is independent of the self-signed CA used for the streaming port (8089).
static bool configureSecureClientForHTTPS(WiFiClientSecure& client) {
  if (!isSystemTimeValidForTLS()) {
    syncSystemTimeFromGPS();
  }
  client.setCACert(ISRG_ROOT_X1_PEM);
  return true;
}

// Attempt to download the TAK server's own (self-signed) CA cert from the
// standard OpenTAK TLS config endpoint and store it at takCACertPath so that
// the streaming connection on port 8089 can verify the server certificate.
static void fetchAndStoreTAKServerCA(const String& enrollHost, uint16_t enrollPort) {
  if (enrollHost.length() == 0) {
    return;
  }

  // Only fetch if we don't already have a server CA stored.
  if (LittleFS.exists(config.takCACertPath)) {
    String existing = readFileAsString(config.takCACertPath);
    if (existing.indexOf("-----BEGIN CERTIFICATE-----") >= 0) {
      Serial.println("TAK: Server CA already stored, skipping fetch.");
      return;
    }
  }

  // Try the two standard OpenTAK paths for the server CA PEM.
  const char* caPaths[] = {
    "/Marti/api/tls/config",
    "/api/tls/config",
    "/Marti/api/tls/config?type=CA",
  };

  WiFiClientSecure caHttps;
  caHttps.setCACert(ISRG_ROOT_X1_PEM); // enrollment server is Let's Encrypt

  for (size_t i = 0; i < sizeof(caPaths) / sizeof(caPaths[0]); ++i) {
    String url = "https://" + enrollHost + ":" + String(enrollPort) + String(caPaths[i]);
    Serial.printf("TAK: Fetching server CA from %s\n", url.c_str());

    HTTPClient http;
    if (!http.begin(caHttps, url)) {
      continue;
    }
    http.addHeader("Accept", "application/json,application/x-pem-file,text/plain");
    int code = http.GET();
    String body = http.getString();
    String ct   = http.header("Content-Type");
    http.end();

    if (code < 200 || code >= 300) {
      Serial.printf("TAK:   CA fetch HTTP %d at %s\n", code, caPaths[i]);
      continue;
    }

    // Accept direct PEM response.
    if (body.indexOf("-----BEGIN CERTIFICATE-----") >= 0) {
      if (writeStringToFile(config.takCACertPath, body)) {
        Serial.println("TAK: Server CA cert stored from direct PEM response.");
      }
      return;
    }

    // Accept JSON response with common field names used by OpenTAK.
    if (ct.indexOf("application/json") >= 0 || body.startsWith("{")) {
      JsonDocument caDoc;
      if (deserializeJson(caDoc, body) == DeserializationError::Ok) {
        const char* caFields[] = {
          "Certificate Authority", "ca", "caCert", "caCertificate",
          "truststore", "signingCert"
        };
        for (size_t f = 0; f < sizeof(caFields) / sizeof(caFields[0]); ++f) {
          String caPem = caDoc[caFields[f]] | "";
          if (caPem.indexOf("-----BEGIN CERTIFICATE-----") >= 0) {
            // Unescape \n sequences that some JSON encoders emit.
            caPem.replace("\\n", "\n");
            if (writeStringToFile(config.takCACertPath, caPem)) {
              Serial.printf("TAK: Server CA cert stored from JSON field '%s'.\n", caFields[f]);
            }
            return;
          }
        }
      }
    }
  }

  Serial.println("TAK: Could not auto-fetch server CA cert.");
  Serial.println("TAK:      For port 8089 streaming: set Verify Certificate=OFF in TAK settings,");
  Serial.println("TAK:      OR manually upload the server CA PEM to the CA cert path.");
}

static void loadClientCertificateIfAvailable() {
  takCertLoaded = false;
  if (!takClientUsesSSL || !config.takUseClientCert) {
    return;
  }

  if (!LittleFS.exists(config.takClientCertPath) || !LittleFS.exists(config.takClientKeyPath)) {
    return;
  }

  String certPem = readFileAsString(config.takClientCertPath);
  String keyPem = readFileAsString(config.takClientKeyPath);
  if (certPem.length() == 0 || keyPem.length() == 0) {
    return;
  }

  takSecureClient.setCertificate(certPem.c_str());
  takSecureClient.setPrivateKey(keyPem.c_str());
  takCertLoaded = true;
}

static void buildAutoUIDIfEmpty() {
  if (strlen(config.takUID) > 0) {
    return;
  }

  String mac = WiFi.macAddress();
  mac.replace(":", "");
  String uid = String(config.deviceName) + "-" + mac;
  uid.toCharArray(config.takUID, sizeof(config.takUID));
}

static String sanitizeHostValue(const String& rawHost) {
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

  int queryIdx = host.indexOf('?');
  if (queryIdx >= 0) {
    host = host.substring(0, queryIdx);
  }

  int hashIdx = host.indexOf('#');
  if (hashIdx >= 0) {
    host = host.substring(0, hashIdx);
  }

  // Strip ':port' for standard hostnames while leaving IPv6-like values untouched.
  int firstColon = host.indexOf(':');
  if (firstColon > 0 && host.indexOf(':', firstColon + 1) < 0) {
    host = host.substring(0, firstColon);
  }

  while (host.endsWith("/")) {
    host.remove(host.length() - 1);
  }

  host.trim();
  return host;
}

static String sanitizeFsPathValue(const String& rawPath) {
  String path = rawPath;
  path.trim();

  if (path.startsWith("/littlefs")) {
    path = path.substring(9);
  } else if (path.startsWith("littlefs/")) {
    path = "/" + path.substring(8);
  }

  if (!path.startsWith("/")) {
    path = "/" + path;
  }

  while (path.indexOf("//") >= 0) {
    path.replace("//", "/");
  }

  return path;
}

static void sanitizeHostBuffer(char* buffer, size_t bufferSize) {
  if (buffer == nullptr || bufferSize == 0) {
    return;
  }
  String sanitized = sanitizeHostValue(String(buffer));
  sanitized.toCharArray(buffer, bufferSize);
}

static void sanitizePathBuffer(char* buffer, size_t bufferSize) {
  if (buffer == nullptr || bufferSize == 0) {
    return;
  }
  String sanitized = sanitizeFsPathValue(String(buffer));
  sanitized.toCharArray(buffer, bufferSize);
}

static String getSanitizedTAKServerHost() {
  return sanitizeHostValue(String(config.takServer));
}

static String getSanitizedEnrollmentHostOrServer() {
  if (strlen(config.takEnrollHost) > 0) {
    return sanitizeHostValue(String(config.takEnrollHost));
  }
  return getSanitizedTAKServerHost();
}

static void normalizeTakConfigPaths() {
  sanitizePathBuffer(config.takCACertPath, sizeof(config.takCACertPath));
  sanitizePathBuffer(config.takClientCertPath, sizeof(config.takClientCertPath));
  sanitizePathBuffer(config.takClientKeyPath, sizeof(config.takClientKeyPath));
  sanitizePathBuffer(config.takClientP12Path, sizeof(config.takClientP12Path));
}

static bool migrateLegacyTakEnrollmentConfigIfNeeded() {
  bool changed = false;

  String enrollPath = String(config.takEnrollPath);
  enrollPath.trim();
  if (enrollPath.equalsIgnoreCase("/Marti/enrollment") ||
      enrollPath.equalsIgnoreCase("Marti/enrollment") ||
      enrollPath.equalsIgnoreCase("/Marti/enrollment?format=pem")) {
    String("/Marti/api/tls/signClient/v2").toCharArray(config.takEnrollPath, sizeof(config.takEnrollPath));
    changed = true;
  }

  return changed;
}

static void applyTLSMode() {
  takClientUsesSSL = config.takSSL;
  takClient = takClientUsesSSL ? static_cast<WiFiClient*>(&takSecureClient)
                               : static_cast<WiFiClient*>(&takPlainClient);

  if (!takClientUsesSSL) {
    return;
  }

  if (config.takVerifyCert) {
    String certData = readFileAsString(config.takCACertPath);
    if (certData.length() > 0) {
      takSecureClient.setCACert(certData.c_str());
      Serial.println("TAK: Using CA cert from LittleFS.");
    } else {
      takSecureClient.setCACert(ISRG_ROOT_X1_PEM);
      Serial.println("TAK: Using embedded ISRG Root X1 cert.");
    }
  } else {
    takSecureClient.setInsecure();
    Serial.println("TAK: TLS insecure mode enabled.");
  }

  loadClientCertificateIfAvailable();
}

void refreshTAKClientMode() {
  bool wasConnected = takClient->connected();
  if (wasConnected) {
    takClient->stop();
  }
  applyTLSMode();
}

static bool connectTAK() {
  if (!config.takEnabled) {
    return false;
  }
  String serverHost = getSanitizedTAKServerHost();
  if (serverHost.length() == 0 || config.takPort == 0) {
    return false;
  }

  if (takClient->connected()) {
    return true;
  }

  if (takClientUsesSSL && config.takVerifyCert && !isSystemTimeValidForTLS()) {
    if (!syncSystemTimeFromGPS()) {
      Serial.println("TAK: Waiting for valid GPS date/time before TLS verify connect.");
      return false;
    }
  }

  // ── Early block: enrollment incomplete (key present, cert missing) ────────
  // Use cached state to avoid LittleFS.exists() spam on every reconnect cycle.
  if (takClientUsesSSL && config.takUseClientCert && !takCertLoaded) {
    if (cachedKeyExists && !cachedCertExists) {
      Serial.println("TAK: Enrollment incomplete (key present, cert missing). Press 'Re-enroll Device'.");
      return false;
    }
  }
  // ─────────────────────────────────────────────────────────────────────────

  Serial.printf("TAK: Connecting to '%s:%u' SSL=%d verify=%d certLoaded=%d\n",
                serverHost.c_str(), (unsigned)config.takPort,
                takClientUsesSSL ? 1 : 0,
                config.takVerifyCert ? 1 : 0,
                takCertLoaded ? 1 : 0);

  bool connected = takClient->connect(serverHost.c_str(), config.takPort);

  if (!connected && takClientUsesSSL) {
    Serial.printf("TAK: Connect failed. Diagnosing (verify=%d)...\n", config.takVerifyCert ? 1 : 0);

    if (config.takVerifyCert) {
      // Try insecure to determine whether this is a CA-trust failure or a
      // network / mTLS-rejection failure.
      takSecureClient.stop();
      takSecureClient.setInsecure();
      bool insecureOk = takClient->connect(serverHost.c_str(), config.takPort);

      // Restore CA cert regardless of outcome.
      String storedCa = readFileAsString(config.takCACertPath);
      if (storedCa.length() > 0) {
        takSecureClient.setCACert(storedCa.c_str());
      } else {
        takSecureClient.setCACert(ISRG_ROOT_X1_PEM);
      }

      if (insecureOk) {
        takSecureClient.stop();
        Serial.println("TAK: Insecure connect succeeded -> server CA is not trusted.");
        if (storedCa.length() == 0) {
          Serial.println("TAK:   No server CA stored yet. Complete enrollment to auto-fetch it,");
          Serial.println("TAK:   OR set 'Verify Certificate' OFF in TAK settings.");
        } else {
          Serial.println("TAK:   Stored CA may not match server. Re-enroll to refresh it.");
        }
      } else {
        Serial.println("TAK: Insecure connect also failed.");
        if (!takCertLoaded) {
          Serial.println("TAK:   Server likely requires a client certificate (mTLS).");
          Serial.println("TAK:   Press 'Re-enroll Device' to obtain one.");
        } else {
          Serial.println("TAK:   Check hostname, port, and server availability.");
        }
      }
    }
  }

  if (!connected) {
    Serial.println("TAK: TCP connection failed.");
  }
  return connected;
}

static String twoDigits(int value) {
  if (value < 10) {
    return "0" + String(value);
  }
  return String(value);
}

static String urlEncode(const String& input) {
  static const char hex[] = "0123456789ABCDEF";
  String encoded;
  encoded.reserve(input.length() * 3);

  for (size_t i = 0; i < input.length(); ++i) {
    char c = input.charAt(i);
    bool isSafe = (c >= 'a' && c <= 'z') ||
                  (c >= 'A' && c <= 'Z') ||
                  (c >= '0' && c <= '9') ||
                  c == '-' || c == '_' || c == '.' || c == '~';
    if (isSafe) {
      encoded += c;
    } else {
      encoded += '%';
      encoded += hex[(c >> 4) & 0x0F];
      encoded += hex[c & 0x0F];
    }
  }

  return encoded;
}

static String buildCotTime(int addSeconds) {
  int year = 1970;
  int month = 1;
  int day = 1;

  String gpsDate = getGPSDate();
  int firstDash = gpsDate.indexOf('-');
  int secondDash = gpsDate.lastIndexOf('-');
  if (firstDash > 0 && secondDash > firstDash) {
    year = gpsDate.substring(0, firstDash).toInt();
    month = gpsDate.substring(firstDash + 1, secondDash).toInt();
    day = gpsDate.substring(secondDash + 1).toInt();
  }

  int rawTime = sensorData.gpsTime;
  int hours = 0;
  int mins = 0;
  int secs = 0;

  if (rawTime > 999999) {
    hours = (rawTime / 1000000) % 100;
    mins = (rawTime / 10000) % 100;
    secs = (rawTime / 100) % 100;
  } else {
    hours = (rawTime / 10000) % 100;
    mins = (rawTime / 100) % 100;
    secs = rawTime % 100;
  }

  if (hours < 0 || hours > 23) hours = 0;
  if (mins < 0 || mins > 59) mins = 0;
  if (secs < 0 || secs > 59) secs = 0;

  secs += addSeconds;
  mins += secs / 60;
  secs %= 60;
  hours += mins / 60;
  mins %= 60;
  hours %= 24;

  return String(year) + "-" + twoDigits(month) + "-" + twoDigits(day) +
         "T" + twoDigits(hours) + ":" + twoDigits(mins) + ":" + twoDigits(secs) + "Z";
}

static String buildCoTMessage() {
  String cotType = strlen(config.takCotType) > 0 ? String(config.takCotType) : String("a-f-G-U-C");
  String callsign = strlen(config.takCallsign) > 0 ? String(config.takCallsign) : String(config.deviceName);
  String uid = strlen(config.takUID) > 0 ? String(config.takUID) : String(config.deviceName);

  String timeStr = buildCotTime(0);
  String staleStr = buildCotTime(config.takIntervalSec * 2);

  String xml;
  xml.reserve(700);
  xml += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
  xml += "<event version=\"2.0\" uid=\"" + uid + "\" type=\"" + cotType +
         "\" time=\"" + timeStr + "\" start=\"" + timeStr +
         "\" stale=\"" + staleStr + "\" how=\"m-g\">";
  xml += "<point lat=\"" + String(sensorData.ownLat, 8) + "\" lon=\"" +
         String(sensorData.ownLon, 8) + "\" hae=\"9999999.0\" ce=\"9999999.0\" le=\"9999999.0\"/>";
  xml += "<detail>";
  xml += "<contact callsign=\"" + callsign + "\"/>";
  xml += "<track speed=\"0.0\" course=\"" + String(sensorData.compassHeading, 2) + "\"/>";
  xml += "<status battery=\"100\"/>";
  xml += "</detail></event>\n";
  return xml;
}

static void sendCoTUDP(const String& xml) {
  if (!config.takUDPEnabled || config.takUDPPort == 0) {
    return;
  }

  if (!takUdpClient.beginPacket("239.2.3.1", config.takUDPPort)) {
    Serial.println("TAK: Failed to begin UDP packet.");
    return;
  }
  takUdpClient.write(reinterpret_cast<const uint8_t*>(xml.c_str()), xml.length());
  takUdpClient.endPacket();
}

static void sendCoTTCP(const String& xml) {
  if (!connectTAK()) {
    return;
  }

  takClient->print(xml);

  if (!config.takPersistent) {
    takClient->stop();
  }
}

void setupTAK() {
  bool configChanged = false;

  ensureCertDir();

  bool migratedLegacy = migrateLegacyTakEnrollmentConfigIfNeeded();
  configChanged = configChanged || migratedLegacy;

  sanitizeHostBuffer(config.takServer, sizeof(config.takServer));
  sanitizeHostBuffer(config.takEnrollHost, sizeof(config.takEnrollHost));

  String caBefore = String(config.takCACertPath);
  String certBefore = String(config.takClientCertPath);
  String keyBefore = String(config.takClientKeyPath);
  String p12Before = String(config.takClientP12Path);
  normalizeTakConfigPaths();
  if (caBefore != String(config.takCACertPath) ||
      certBefore != String(config.takClientCertPath) ||
      keyBefore != String(config.takClientKeyPath) ||
      p12Before != String(config.takClientP12Path)) {
    configChanged = true;
  }

  if (configChanged) {
    saveConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str());
    Serial.println("TAK: Migrated and saved TAK config defaults/paths.");
  }

  buildAutoUIDIfEmpty();
  applyTLSMode();
  refreshCertFileCache();

  Serial.printf("TAK: Effective enroll host='%s' port=%u path='%s'\n",
                config.takEnrollHost, (unsigned)config.takEnrollPort, config.takEnrollPath);
  Serial.printf("TAK: Effective client cert path='%s' key path='%s' ca path='%s'\n",
                config.takClientCertPath, config.takClientKeyPath, config.takCACertPath);

  lastTAKSendMs = millis();
  lastReconnectAttemptMs = 0;
}

void handleTAK() {
  if (takConnectRequested) {
    takConnectRequested = false;
    connectTAK();
  }

  if (takDisconnectRequested) {
    takDisconnectRequested = false;
    if (takClient->connected()) {
      takClient->stop();
    }
  }

  if (!config.takEnabled || config.takIntervalSec == 0) {
    return;
  }

  if (config.takPersistent && !config.takUDPEnabled && !takClient->connected()) {
    unsigned long nowMs = millis();
    if (nowMs - lastReconnectAttemptMs >= TAK_RECONNECT_BACKOFF_MS) {
      lastReconnectAttemptMs = nowMs;
      connectTAK();
    }
  }

  unsigned long now = millis();
  unsigned long intervalMs = static_cast<unsigned long>(config.takIntervalSec) * 1000UL;
  if (now - lastTAKSendMs < intervalMs) {
    return;
  }

  String cot = buildCoTMessage();
  if (getSanitizedTAKServerHost().length() > 0) {
    sendCoTTCP(cot);
  }
  sendCoTUDP(cot);
  lastTAKSendMs = now;
}

bool isTAKConnected() {
  return takClient->connected();
}

unsigned long getLastTAKSendMs() {
  return lastTAKSendMs;
}

String getTAKEnrollmentURI() {
  String host = getSanitizedEnrollmentHostOrServer();
  String user = String(config.takEnrollUsername);
  String token = String(config.takEnrollToken);

  if (host.length() == 0 || user.length() == 0 || token.length() == 0) {
    return "";
  }

  return "tak://com.atakmap.app/enroll?host=" + urlEncode(host) +
         "&username=" + urlEncode(user) +
         "&token=" + urlEncode(token);
}

bool refreshTAKEnrollmentToken(String& message) {
  String host = getSanitizedEnrollmentHostOrServer();
  String apiPath = String(config.takTokenApiPath);
  if (host.length() == 0 || apiPath.length() == 0) {
    message = "Missing takEnrollHost/takServer or takTokenApiPath";
    return false;
  }

  if (!apiPath.startsWith("/")) {
    apiPath = "/" + apiPath;
  }

  WiFiClientSecure httpsClient;
  configureSecureClientForHTTPS(httpsClient);
  HTTPClient http;

  String url = "https://" + host + ":" + String(config.takEnrollPort) + apiPath;
  if (!http.begin(httpsClient, url)) {
    message = "Token API begin failed";
    return false;
  }

  if (strlen(config.takTokenApiUsername) > 0) {
    http.setAuthorization(config.takTokenApiUsername, config.takTokenApiPassword);
  }
  http.addHeader("Accept", "application/json");

  int code = http.GET();
  String body = http.getString();
  http.end();

  if (code < 200 || code >= 300) {
    message = "Token API HTTP " + String(code);
    return false;
  }

  JsonDocument tokenDoc;
  DeserializationError err = deserializeJson(tokenDoc, body);
  if (err) {
    message = "Token API JSON parse failed";
    return false;
  }

  String token = tokenDoc["token"] | tokenDoc["jwt"] | tokenDoc["enrollmentToken"] | "";
  String username = tokenDoc["username"] | tokenDoc["user"] | tokenDoc["slot"] | "";

  if (token.length() == 0) {
    message = "Token API response did not include token";
    return false;
  }

  if (username.length() > 0) {
    username.toCharArray(config.takEnrollUsername, sizeof(config.takEnrollUsername));
  }
  token.toCharArray(config.takEnrollToken, sizeof(config.takEnrollToken));

  saveConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str());
  message = "Token refreshed";
  return true;
}

static bool generateKeyAndCSR(String& keyPem, String& csrPem, String& message) {
  mbedtls_pk_context pk;
  mbedtls_ctr_drbg_context ctr;
  mbedtls_entropy_context entropy;
  mbedtls_x509write_csr req;
  String subject;
  String csrCommonName;
  unsigned char keyBuf[2200] = {0};
  unsigned char csrBuf[2200] = {0};
  int rc = 0;
  bool ok = false;

  mbedtls_pk_init(&pk);
  mbedtls_ctr_drbg_init(&ctr);
  mbedtls_entropy_init(&entropy);
  mbedtls_x509write_csr_init(&req);

  do {
    const char* pers = "tak-enroll";
    rc = mbedtls_ctr_drbg_seed(&ctr, mbedtls_entropy_func, &entropy,
                               reinterpret_cast<const unsigned char*>(pers), strlen(pers));
    if (rc != 0) {
      message = "CSR RNG seed failed";
      break;
    }

    rc = mbedtls_pk_setup(&pk, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY));
    if (rc != 0) {
      message = "PK setup failed";
      break;
    }

    rc = mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, mbedtls_pk_ec(pk),
                             mbedtls_ctr_drbg_random, &ctr);
    if (rc != 0) {
      message = "EC key generation failed";
      break;
    }

    // OTS enrollment expects CSR CN to match the auth username.
    csrCommonName = strlen(config.takEnrollUsername) > 0
      ? String(config.takEnrollUsername)
      : String(config.takUID);
    if (csrCommonName.length() == 0) {
      message = "CSR CN missing (set Enroll Username or UID)";
      break;
    }

    subject = "CN=" + csrCommonName;
    mbedtls_x509write_csr_set_md_alg(&req, MBEDTLS_MD_SHA256);
    mbedtls_x509write_csr_set_key(&req, &pk);
    rc = mbedtls_x509write_csr_set_subject_name(&req, subject.c_str());
    if (rc != 0) {
      message = "CSR subject set failed";
      break;
    }

    rc = mbedtls_pk_write_key_pem(&pk, keyBuf, sizeof(keyBuf));
    if (rc != 0) {
      message = "Key PEM write failed";
      break;
    }
    keyPem = String(reinterpret_cast<char*>(keyBuf));

    rc = mbedtls_x509write_csr_pem(&req, csrBuf, sizeof(csrBuf), mbedtls_ctr_drbg_random, &ctr);
    if (rc != 0) {
      message = "CSR PEM write failed";
      break;
    }
    csrPem = String(reinterpret_cast<char*>(csrBuf));

    message = "CSR generated";
    ok = true;
  } while (false);

  mbedtls_x509write_csr_free(&req);
  mbedtls_pk_free(&pk);
  mbedtls_ctr_drbg_free(&ctr);
  mbedtls_entropy_free(&entropy);

  return ok;
}

bool enrollTAKClientCertificate(bool forceReenroll, String& message) {
  lastEnrollAttemptMs = millis();

  if (forceReenroll) {
    if (LittleFS.exists(config.takClientCertPath)) LittleFS.remove(config.takClientCertPath);
    if (LittleFS.exists(config.takClientKeyPath))  LittleFS.remove(config.takClientKeyPath);
    if (LittleFS.exists(config.takClientP12Path))  LittleFS.remove(config.takClientP12Path);
    refreshCertFileCache();
  }

  if (!forceReenroll && LittleFS.exists(config.takClientCertPath) && LittleFS.exists(config.takClientKeyPath)) {
    loadClientCertificateIfAvailable();
    message = "Client cert already present";
    lastEnrollMessage = message;
    return true;
  }

  String keyPem;
  String csrPem;
  if (!generateKeyAndCSR(keyPem, csrPem, message)) {
    lastEnrollMessage = message;
    return false;
  }

  if (!writeStringToFile(config.takClientKeyPath, keyPem)) {
    message = "Failed to store client private key";
    lastEnrollMessage = message;
    return false;
  }

  String host = getSanitizedEnrollmentHostOrServer();
  if (host.length() == 0) {
    message = "Missing enrollment host";
    lastEnrollMessage = message;
    return false;
  }

  String enrollPath = String(config.takEnrollPath);
  if (!enrollPath.startsWith("/")) {
    enrollPath = "/" + enrollPath;
  }

  // OTS signClient endpoint requires clientUid in query params.
  if (enrollPath.indexOf("signClient/v2") >= 0 && enrollPath.indexOf("clientUid=") < 0) {
    String clientUid = String(config.takUID);
    if (clientUid.length() > 0) {
      enrollPath += (enrollPath.indexOf("?") >= 0) ? "&" : "?";
      enrollPath += "clientUid=" + clientUid;
    }
  }

  if (enrollPath.indexOf("?") < 0) {
    enrollPath += "?format=pem";
  }

  const bool isSignClientV2 = enrollPath.indexOf("signClient/v2") >= 0;
  const bool hasEnrollUser = strlen(config.takEnrollUsername) > 0;
  const bool hasEnrollToken = strlen(config.takEnrollToken) > 0;

  // OTS signClient flow requires Basic auth (username + token/password).
  if (isSignClientV2 && (!hasEnrollUser || !hasEnrollToken)) {
    message = "Missing Enroll Username/Token for /Marti/api/tls/signClient/v2";
    lastEnrollMessage = message;
    Serial.printf("TAK: Enrollment blocked: signClient/v2 requires username+token. user_len=%u token_len=%u\n",
                  (unsigned)strlen(config.takEnrollUsername),
                  (unsigned)strlen(config.takEnrollToken));
    return false;
  }

  WiFiClientSecure httpsClient;
  // Enrollment bootstrap: server CA may not be available yet, so use TOFU.
  // After enrollment we store ca0 and use that for subsequent TLS verification.
  httpsClient.setInsecure();
  Serial.println("TAK: Enrollment HTTPS using insecure bootstrap mode (TOFU).");
  HTTPClient http;
  String url = "https://" + host + ":" + String(config.takEnrollPort) + enrollPath;

  if (!http.begin(httpsClient, url)) {
    message = "Enrollment HTTP begin failed";
    lastEnrollMessage = message;
    return false;
  }

  if (enrollPath.indexOf("signClient/v2") >= 0) {
    http.addHeader("Content-Type", "application/octet-stream");
    http.addHeader("Accept", "application/json");
  } else {
    http.addHeader("Content-Type", "application/pkcs10");
    http.addHeader("Accept", "application/x-pem-file,application/json,application/octet-stream,application/pkcs12");
  }

  // Send enrollment token as Basic auth (username:token) if configured.
  // OpenTAK requires this for the enrollment endpoint.
  if (hasEnrollUser && hasEnrollToken) {
    http.setAuthorization(config.takEnrollUsername, config.takEnrollToken);
    Serial.printf("TAK: Enrollment auth: user='%s' token_len=%u\n",
                  config.takEnrollUsername, (unsigned)strlen(config.takEnrollToken));
  } else if (hasEnrollToken) {
    // Token-only: try as Bearer token
    http.addHeader("Authorization", "Bearer " + String(config.takEnrollToken));
    Serial.printf("TAK: Enrollment Bearer token len=%u\n", (unsigned)strlen(config.takEnrollToken));
  } else {
    Serial.println("TAK: WARNING -- no enrollment credentials set (token/username empty).");
    Serial.println("TAK:   Set Enroll Username + Enroll Token in TAK settings.");
  }

  Serial.printf("TAK: POSTing CSR to %s\n", url.c_str());
  uint8_t* csrPayload = reinterpret_cast<uint8_t*>(const_cast<char*>(csrPem.c_str()));
  int code = http.POST(csrPayload, csrPem.length());
  Serial.printf("TAK: Enrollment response HTTP %d\n", code);
  if (code < 200 || code >= 300) {
    String errBody = http.getString();
    http.end();
    Serial.printf("TAK: Enrollment error body (first 256): %.256s\n", errBody.c_str());
    if (code == 401 || code == 403) {
      message = "Enrollment HTTP " + String(code) + " -- check Enroll Username/Token in TAK settings";
    } else if (code == 404) {
      message = "Enrollment HTTP 404 -- check Enroll Path in TAK settings (default: /Marti/api/tls/signClient/v2)";
    } else if (code == 400 && errBody.indexOf("No required SSL certificate was sent") >= 0) {
      message = "Enrollment endpoint requires client TLS cert at proxy. Use public OTS enroll host or relax nginx client cert requirement for signClient/v2";
    } else {
      message = "Enrollment HTTP " + String(code);
    }
    lastEnrollMessage = message;
    return false;
  }

  String contentType = http.header("Content-Type");
  String body = http.getString();
  http.end();

  bool storedPem = false;
  bool storedCaPem = false;
  if (body.indexOf("-----BEGIN CERTIFICATE-----") >= 0) {
    storedPem = writeStringToFile(config.takClientCertPath, body);
  } else if (contentType.indexOf("application/json") >= 0) {
    JsonDocument responseDoc;
    if (deserializeJson(responseDoc, body) == DeserializationError::Ok) {
      String certValue = responseDoc["signedCert"] | responseDoc["certificate"] |
                         responseDoc["certPem"] | responseDoc["clientCertificate"] | "";
      String certPem = normalizeCertificateFromApiField(certValue);
      if (certPem.indexOf("-----BEGIN CERTIFICATE-----") >= 0) {
        storedPem = writeStringToFile(config.takClientCertPath, certPem);
      }

      String caValue = responseDoc["ca0"] | responseDoc["ca"] |
                       responseDoc["caCert"] | responseDoc["caCertificate"] | "";
      String caPem = normalizeCertificateFromApiField(caValue);
      if (caPem.indexOf("-----BEGIN CERTIFICATE-----") >= 0) {
        storedCaPem = writeStringToFile(config.takCACertPath, caPem);
        if (storedCaPem) {
          Serial.println("TAK: Stored enrollment CA cert (ca0) from JSON response.");
        }
      }
    }
  }

  if (!storedPem) {
    if (body.length() > 0) {
      writeStringToFile(config.takClientP12Path, body);
      refreshCertFileCache();
    }
    Serial.printf("TAK: Enrollment non-PEM body (first 256): %.256s\n", body.c_str());
    message = "Enrollment response did not contain PEM cert (stored raw response)";
    lastEnrollMessage = message;
    return false;
  }

  refreshCertFileCache();
  loadClientCertificateIfAvailable();

  // After obtaining a client cert, also fetch and store the server's own CA
  // cert so that the streaming connection (port 8089, self-signed CA) can
  // verify the server.  Uses the enrollment host because that API is on the
  // same Let's Encrypt-backed server.
  if (takCertLoaded && !storedCaPem) {
    String enrollHost = getSanitizedEnrollmentHostOrServer();
    fetchAndStoreTAKServerCA(enrollHost, config.takEnrollPort);
    // Reload TLS mode so the newly stored CA is picked up immediately.
    refreshTAKClientMode();
  }

  message = takCertLoaded ? "Enrollment complete" : "Cert stored but not loaded";
  lastEnrollMessage = message;
  return takCertLoaded;
}

void getTAKEnrollmentStatus(JsonDocument& statusDoc) {
  // Use cached state — avoids LittleFS.exists() on every UI poll.
  statusDoc["enrolled"]  = takCertLoaded;
  statusDoc["certExists"] = cachedCertExists;
  statusDoc["keyExists"]  = cachedKeyExists;
  statusDoc["p12Exists"]  = cachedP12Exists;
  statusDoc["lastEnrollMessage"] = lastEnrollMessage;
  statusDoc["lastEnrollMs"] = lastEnrollAttemptMs;
}

void mergeTAKConfigPatch(const JsonDocument& patchDoc) {
  JsonObjectConst obj = patchDoc.as<JsonObjectConst>();

  if (obj["takEnabled"].is<bool>()) config.takEnabled = obj["takEnabled"].as<bool>();
  if (obj["takSSL"].is<bool>()) config.takSSL = obj["takSSL"].as<bool>();
  if (obj["takVerifyCert"].is<bool>()) config.takVerifyCert = obj["takVerifyCert"].as<bool>();
  if (obj["takPersistent"].is<bool>()) config.takPersistent = obj["takPersistent"].as<bool>();
  if (obj["takUDPEnabled"].is<bool>()) config.takUDPEnabled = obj["takUDPEnabled"].as<bool>();

  if (obj["takServer"].is<const char*>()) {
    String(obj["takServer"].as<const char*>()).toCharArray(config.takServer, sizeof(config.takServer));
  }
  if (obj["takPort"].is<int>()) config.takPort = obj["takPort"].as<int>();
  if (obj["takUDPPort"].is<int>()) config.takUDPPort = obj["takUDPPort"].as<int>();
  if (obj["takCallsign"].is<const char*>()) {
    String(obj["takCallsign"].as<const char*>()).toCharArray(config.takCallsign, sizeof(config.takCallsign));
  }
  if (obj["takUID"].is<const char*>()) {
    String(obj["takUID"].as<const char*>()).toCharArray(config.takUID, sizeof(config.takUID));
  }
  if (obj["takCotType"].is<const char*>()) {
    String(obj["takCotType"].as<const char*>()).toCharArray(config.takCotType, sizeof(config.takCotType));
  }
  if (obj["takIntervalSec"].is<int>()) config.takIntervalSec = obj["takIntervalSec"].as<int>();
  if (obj["takCACertPath"].is<const char*>()) {
    String(obj["takCACertPath"].as<const char*>()).toCharArray(config.takCACertPath, sizeof(config.takCACertPath));
  }

  if (obj["takEnrollPort"].is<int>()) config.takEnrollPort = obj["takEnrollPort"].as<int>();
  if (obj["takEnrollHost"].is<const char*>()) {
    String(obj["takEnrollHost"].as<const char*>()).toCharArray(config.takEnrollHost, sizeof(config.takEnrollHost));
  }
  sanitizeHostBuffer(config.takServer, sizeof(config.takServer));
  sanitizeHostBuffer(config.takEnrollHost, sizeof(config.takEnrollHost));
  normalizeTakConfigPaths();
  if (obj["takEnrollUsername"].is<const char*>()) {
    String(obj["takEnrollUsername"].as<const char*>()).toCharArray(config.takEnrollUsername, sizeof(config.takEnrollUsername));
  }
  if (obj["takEnrollToken"].is<const char*>()) {
    String(obj["takEnrollToken"].as<const char*>()).toCharArray(config.takEnrollToken, sizeof(config.takEnrollToken));
  }
  if (obj["takAutoTokenFetch"].is<bool>()) config.takAutoTokenFetch = obj["takAutoTokenFetch"].as<bool>();
  if (obj["takTokenApiPath"].is<const char*>()) {
    String(obj["takTokenApiPath"].as<const char*>()).toCharArray(config.takTokenApiPath, sizeof(config.takTokenApiPath));
  }
  if (obj["takTokenApiUsername"].is<const char*>()) {
    String(obj["takTokenApiUsername"].as<const char*>()).toCharArray(config.takTokenApiUsername, sizeof(config.takTokenApiUsername));
  }
  if (obj["takTokenApiPassword"].is<const char*>()) {
    String(obj["takTokenApiPassword"].as<const char*>()).toCharArray(config.takTokenApiPassword, sizeof(config.takTokenApiPassword));
  }
  if (obj["takUseClientCert"].is<bool>()) config.takUseClientCert = obj["takUseClientCert"].as<bool>();
  if (obj["takEnrollPath"].is<const char*>()) {
    String(obj["takEnrollPath"].as<const char*>()).toCharArray(config.takEnrollPath, sizeof(config.takEnrollPath));
  }
  if (obj["takClientCertPath"].is<const char*>()) {
    String(obj["takClientCertPath"].as<const char*>()).toCharArray(config.takClientCertPath, sizeof(config.takClientCertPath));
  }
  if (obj["takClientKeyPath"].is<const char*>()) {
    String(obj["takClientKeyPath"].as<const char*>()).toCharArray(config.takClientKeyPath, sizeof(config.takClientKeyPath));
  }
  if (obj["takClientP12Path"].is<const char*>()) {
    String(obj["takClientP12Path"].as<const char*>()).toCharArray(config.takClientP12Path, sizeof(config.takClientP12Path));
  }

  refreshTAKClientMode();
}
