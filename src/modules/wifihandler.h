#ifndef WIFIHANDLER_H
#define WIFIHANDLER_H

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <LittleFS.h>
#include "../structs/config.h" // Include the config.h file

extern Config config;
extern String jsonDir;
extern String fileConfigJSON;
extern String ipAddress;
extern String hostAddress;
extern int intCounterWifi;

void saveConfiguration(FS &fs, const char * path);
void wifiSetup();

#endif