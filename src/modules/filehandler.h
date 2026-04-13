#ifndef FILEHANDLER_H
#define FILEHANDL

#include <LittleFS.h>
#include <ArduinoJson.h>

#include "../structs/config.h"
#include "../structs/caldata.h"
#include "../structs/sensordata.h"
#include "../structs/debugsettings.h"
#include "../structs/selectedmap.h"
#include "../structs/waypointsmap.h"

void putJSONConfigDataInMemory();
void putJSONCalibrationDataInMemory();
void putJSONDebugSettingsInMemory();
void saveCalibrationData(fs::FS &fs, const char * path, const CalData &caldata);
void saveDebugSettings(fs::FS &fs, const char * path, const DebugSettings &debugSettings);
void saveSensorData(fs::FS &fs, const char * path, const SensorData &sensorData);
void saveConfiguration(fs::FS &fs, const char * path);

void addMaptoDB(String PNGFile, String KMLFile, JsonObject obj);
void updateMaptoDB(String PNGFile, String KMLFile, JsonObject obj, bool pngUpdated, bool kmlUpdated);
String prepMapNameForMapDB(String fileName);

void readMapWaypointsFromJSON(fs::FS &fs, const char * path, int requestedMap);
void readMapFromJSON(fs::FS &fs, const char * path, int requestedMap);
void saveSensorDataToJSON();
void saveCalibrationDataToJSON();

String listDir(fs::FS &fs, const char * dirname, uint8_t levels);
String printFreeSpace();
void deleteFile(fs::FS &fs, const char * path);
void getJSandCSSFiles(fs::FS &fs, const char * dirname, uint8_t levels);

void IRAM_ATTR loadConfiguration(fs::FS &fs, const char *path, Config config);
void IRAM_ATTR loadDebugSettings(fs::FS &fs, const char * path, DebugSettings &debugSettings);
void IRAM_ATTR loadCalibrationData(fs::FS &fs, const char * path, CalData &caldata);
void IRAM_ATTR loadSensorData(fs::FS &fs, const char * path, SensorData &sensorData);
void IRAM_ATTR readAllMapsFromJSON(fs::FS &fs, const char * path);

#endif