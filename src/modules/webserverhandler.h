#ifndef WEBSERVERHANDLER_H
#define WEBSERVERHANDLER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "ESPAsyncWebServer.h"
#include <ElegantOTA.h>
#include <DNSServer.h>
#include "filehandler.h"
#include "gps.h"
#include "takhandler.h"

void webServerSetup();

#endif // WEBSERVERHANDLER_H