#ifndef TAKHANDLER_H
#define TAKHANDLER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "../structs/sensordata.h"

void setupTAK();
void getTAKConfig(JsonDocument &doc);
void getTAKStatus(JsonDocument &doc);
void getTAKCertDiagnostics(JsonDocument &doc);
bool updateTAKConfigFromJson(const JsonDocument &doc, String &message);
bool importTAKPackageData(const JsonDocument &doc, String &message);
bool connectTAK(String &message);
void disconnectTAK(const String &reason = String());
void serviceTAK(const SensorData &sensorData);

#endif