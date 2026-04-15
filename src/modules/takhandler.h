#ifndef TAKHANDLER_H
#define TAKHANDLER_H

#include <Arduino.h>
#include <ArduinoJson.h>

void setupTAK();
void handleTAK();
void refreshTAKClientMode();

bool isTAKConnected();
unsigned long getLastTAKSendMs();
String getTAKEnrollmentURI();
bool refreshTAKEnrollmentToken(String& message);
bool enrollTAKClientCertificate(bool forceReenroll, String& message);
void getTAKEnrollmentStatus(JsonDocument& statusDoc);

void mergeTAKConfigPatch(const JsonDocument& patchDoc);

extern bool takConnectRequested;
extern bool takDisconnectRequested;

#endif
