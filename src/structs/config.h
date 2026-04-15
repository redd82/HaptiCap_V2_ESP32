#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

struct Config {
  uint8_t http_port = 80;
  uint8_t dns_port = 53;
  bool asAP = false;
  char clientSSID[25] = "Get_out_of_my_laboratory";
  char clientPasswd[16] = "T0sh7b49";
  int connectionTimeOut = 120;
  char deviceName[16] = "HaptiCap";                       //  = "HCR-99_HaptiCap"
  char apPasswd[16] = "prutser00";
  char http_username[16] = "admin";
  char http_password[16] = "admin";
  unsigned long gpsPollSec = 1;              // Timer0 tick set to 1 s (1 * 1000 = 1000 ms) for gps 
  unsigned long compPollMs = 100;            // Timer1 tick set to 1 ms (100 * 1 = 100 ms)  for compass 
  float compOffset = 123.0;
  float HOME_LAT = 12.234567;
  float HOME_LON = 56.789012;
  float WAYPOINT_LAT = 2.234567;
  float WAYPOINT_LON = 6.789012;
  int targetReached = 10;
  int maxDistance = 500;
  int maxDelay = 1000;
  double declAngleRad = 0.024085543678;
  unsigned long sleepMins = 5;                // Timer2 tick set to 1 min (1 * 60000 = 1 min)  sleep
  int touchThreshold = 50;
  int timeZoneOffset = 1;                      // +1 hour
  bool touchEnabled = true;
  int selectedMap = 1;

  // TAK/OpenTAK settings
  bool takEnabled = false;
  bool takSSL = true;
  bool takVerifyCert = true;
  bool takPersistent = true;
  bool takUDPEnabled = false;
  char takServer[64] = "";
  uint16_t takPort = 8089;
  uint16_t takUDPPort = 6969;
  char takCallsign[32] = "HaptiCap";
  char takUID[48] = "";
  char takCotType[32] = "a-f-G-U-C";
  uint16_t takIntervalSec = 30;
  char takCACertPath[64] = "/certs/tak_ca.crt";

  // ATAK QR enrollment deep-link settings (manual mode)
  uint16_t takEnrollPort = 8446;
  char takEnrollHost[64] = "";
  char takEnrollUsername[64] = "";
  char takEnrollToken[1024] = "";

  // Optional OpenTAK API token refresh settings
  bool takAutoTokenFetch = false;
  char takTokenApiPath[96] = "/api/tokens/current";
  char takTokenApiUsername[64] = "";
  char takTokenApiPassword[64] = "";

  // Device enrollment certificate settings
  bool takUseClientCert = true;
  char takEnrollPath[96] = "/Marti/api/tls/signClient/v2";
  char takClientCertPath[64] = "/certs/tak_client.crt";
  char takClientKeyPath[64] = "/certs/tak_client.key";
  char takClientP12Path[64] = "/certs/tak_client.p12";
};

#endif