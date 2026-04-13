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
};

#endif