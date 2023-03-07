#include <Arduino.h>

#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include <DNSServer.h>
#include "FS.h"
#include <LittleFS.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>
#include <MPU9250_WE.h>
#include <Wire.h>
#include <string>
#include "functions.h"

#define FORMAT_LITTLEFS_IF_FAILED true
#define SWVERSION 2.001
#define MPU9250_ADDR 0x68
#define MAX_SRV_CLIENTS 2
#define INPUT_SIZE 20
#define CONFIG_JSON_DOCSIZE 1024
#define CALDATA_JSON_DOCSIZE 768
#define DEBUGSETTINGS_JSON_DOCSIZE 768
#define SENSORDATA_JSON_DOCSIZE 1024
#define MAPRECIEVED_JSON_DOCSIZE 256
#define MAPLIST_JSON_DOCSIZE 1536
#define NROFWAYPOINTS 20
#define NROFMAPS 8

void setup();
void setupIO();
void webServerFunctions();
void loop();

// Set declination angle on your location and fix heading
// Formula: (deg + (minutes / 60.0)) / (180 / M_PI); (4.0 + (26.0 / 60.0)) / (180 / PI);
//float declinationAngle = (declAngleDeg + (declAngleMin / 60.0)) / (180.0 / PI);
// _Config_
struct Config {
  uint8_t http_port = 80;
  uint8_t dns_port = 53;
  bool asAP = false;
  char clientSSID[16] = "TrizNet_AP2";
  char clientPasswd[16] = "T0sh7b49";
  int connectionTimeOut = 120;
  char deviceName[16] = "HaptiCap";                       //  = "HCR-99_HaptiCap"
  char apPasswd[16] = "prutser00";
  char http_username[16] = "admin";
  char http_password[16] = "admin";
  unsigned long gpsPollSec = 1;               // Timer0 tick set to 1 s (10 * 1000 = 10000 ms) for gps 
  unsigned long compPollMs = 100;            // Timer1 tick set to 1 ms (250 * 1 = 250 ms)  for compass 
  float compOffset = 123.0;
  float HOME_LAT = 12.234567;
  float HOME_LON = 56.789012;
  float WAYPOINT_LAT = 2.234567;
  float WAYPOINT_LON = 6.789012;
  int targetReached = 10;
  int maxDistance = 500;
  int maxDelay = 1000;
  double declAngleRad = 0.024085543678;
  unsigned long sleepMins = 5;                // Timer2 tick set to 1 min (1 * 60000 = 250 ms)  sleep
  int touchThreshold = 50;
  int timeZoneOffset = 1;                      // +1 hour
  bool touchEnabled = true;
  bool ftpEnabled = false;
};

// _CalData_
struct CalData {
  float magBiasX = 0.0;
  float magBiasY = 0.0;
  float magBiasZ = 0.0;
  float magScaleFacX = 1.0;
  float magScaleFacY = 1.0;
  float magScaleFacZ = 1.0;
  float gyroBiasX = 0.0;
  float gyroBiasY = 0.0;
  float gyroBiasZ = 0.0;
  float accelBiasX = 0.0;
  float accelBiasY = 0.0;
  float accelBiasZ = 0.0;
  float accelScaleX = 1.0;
  float accelScaleY = 1.0;
  float accelScaleZ = 1.0;
};

struct DebugSettings {
  bool debugHaptic = false;
  bool debug2Serial = false;
  bool debugGPS2Serial = false;
};

struct SensorData {
  char sensorName[8] = "";
  int gpsTime = 0;
  double ownLat = 0.0;
  double ownLon= 0.0;
  float compassHeading = 0.0;
  char compassCardinal[8] = "";
  double homeBaseLat= 0.0;
  double homeBaseLon= 0.0;
  float homeBaseBearing= 0.0;
  char homeBaseCardinal[8] = "";
  int homeBaseDistance= 0;
  float relheading = 0.0;
  float relheadingHomeBase = 0.0;
  double wayPointLat = 0.0;
  double wayPointLon = 0.0;
  float wayPointBearing= 0.0;
  char wayPointCardinal[8] = "";
  int wayPointDistance = 0;
  int nrOfSatellites = 0;
};

struct Waypoints {
  float homeBase [2];
  float wayPoints [NROFWAYPOINTS][3];
};

struct SelectedMap {
  // id, name, country, area, mapfile, kmlfile
  int map_id; // 1
  char map_name[16]; // "Test area"
  char map_area[16]; // "Kaag en Braassem"
  char map_country[16]; // "Netherlands"
  char map_pngFile[16]; // "kaagenbraassem.png"
  char map_kmlFile[16]; // "kaagenbraassem.kml"
};

StaticJsonDocument<CONFIG_JSON_DOCSIZE> configDoc;
StaticJsonDocument<CALDATA_JSON_DOCSIZE> calDataDoc;
StaticJsonDocument<DEBUGSETTINGS_JSON_DOCSIZE> debugSettingsDoc;
StaticJsonDocument<SENSORDATA_JSON_DOCSIZE> sensorDataDoc;
StaticJsonDocument <MAPLIST_JSON_DOCSIZE> mapListDoc;
StaticJsonDocument <MAPRECIEVED_JSON_DOCSIZE> selectedMapJSON;
JsonArray maps = mapListDoc.createNestedArray("maps");
JsonObject maps_1 = maps.createNestedObject();
JsonObject maps_2 = maps.createNestedObject();
JsonObject maps_3 = maps.createNestedObject();
JsonObject maps_4 = maps.createNestedObject();
JsonObject maps_5 = maps.createNestedObject();
JsonObject maps_6 = maps.createNestedObject();
JsonObject maps_7 = maps.createNestedObject();
JsonObject maps_8 = maps.createNestedObject();

// {
//   "maps": [
//     {
//       "id": 1,
//       "name": "Test area",
//       "area": "Kaag en Braassem",
//       "country": "Netherlands",
//       "pngFile": "/maps/Home.png",
//       "kmlFile": "/maps/Home.kml"
//     }
//   ]
// }

Config config;                         // <- global configuration object
CalData caldata;
DebugSettings debugSettings;
SensorData sensorData;
Waypoints wayPoints;
SelectedMap selectedMap;

// _PARAMS_
String fileConfigJSON = "/hapticap.json";
String fileCalDataJSON = "/caldata.json";
String fileDebugJSON = "/debug.json";
String fileSensorDataJSON = "/sensordata.json";
String fileWayPointDataJSON = "/waypoints.json";
String fileMapDataJSON = "/mapData.json";
String fileJs = "/";
String fileJsMap = "/";
String fileCss = "/";
String fileCssMap = "/";
bool cssJsFileNamesConcat = false;
String rootDir = "/";
String mapsDir = "/maps";
String appDir = "/app";
String jsonDir = "/json";
String uploadedPNGFile = "";
String uploadedKMLFile = "";
String selectedMapPNG = "";
String selectedMapKML = "";
String requestedMap = "";
String currentMap = "";
String previousMap = "";
int mapSelector = 1;
static const uint32_t GPSBaud = 9600;
static const uint32_t SerialUSBBaud = 115200;
static int taskCore = 0;
char compassCardinal[8];
char waypointCardinal[8];
char cardinal2home[8];

// _Interrupts_
volatile int interrupt0;
volatile int interrupt1;
volatile int interrupt2;
volatile int totalInterruptCounter;

// _Timers_
hw_timer_t *timer0 = NULL;
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;
portMUX_TYPE timer0Mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timer1Mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timer2Mux = portMUX_INITIALIZER_UNLOCKED;

const int dta_rdy_pin = 19;
const int led = 23;
const int buttonPin = 4;
const int TouchPinT0 = 4;
const int TouchPinT3 = 15;
int buttonState = 0;
touch_pad_t touchPin;
int touchValueT0;
int touchValueT3;

// PWM settings for haptic feedback
const int distancePOI_scalar = 2;
const int hapticfreq = 5000;
const int resolution = 8;
const int hapticfront = 25;
const int hapticright = 26;
const int hapticrear = 32;
const int hapticleft = 33;
const int pwmhapticfront = 0;
const int pwmhapticright = 1;
const int pwmhapticrear = 2;
const int pwmhapticleft = 3;
int pwm_front = 0;
int pwm_right = 0;
int pwm_rear = 0;
int pwm_left = 0;
bool bPrintHeader = false;
int count = 0;
int i = 0;

RTC_DATA_ATTR double flCurrentLat;
RTC_DATA_ATTR double flCurrentLon;
RTC_DATA_ATTR bool bDumpConfig = 1;
RTC_DATA_ATTR bool bTargetReachedAck = 0;
RTC_DATA_ATTR bool bHomeReachedAck = 0;

String str2HTML;

float declAngleDeg = 1.0;
float declAngleMin = 23.0;
float compassheading = 0.0;
float headingraw = 0.0;
float compassheadingtemp = 0.0;
int compasscounter = 0;
int comp_samples = 50;
float previous_compassheading = 0.0;
float ref_previous_compassheading = 0.0;
float compass_diff = 0.0;
int nrsatt = 0;
float b = (255.0/90.0);   // 256/90 pwm scaled to 90 degrees (quadrant)

String ipAddress;
String hostAddress;
String GPSTimeMinsSecs;
String GPSTimeMins;
String GPSDate;
int hours;
int mins;
int secs;
int day;
int month;
int year;
int intCounterWifi = 0;
bool bUseTimerInterrupt = 1;
bool bYouRang = 0;
bool bNoJSONfile = 0;
bool bJSONnotvalid = 0;
bool haptictouchT0 = 0;
bool haptictouchT3 = 0;
bool restart_esp = 0;
bool hapticfeedback = 0;
bool bSaveConfig = 0;
bool bReportCore0 = 0;
bool bReportCore1 = 0;
bool timers_disabled = 0;
bool bDebugTgrtReached = 0;
bool bDebugHomeReached = 0;
int GPSFix = 0;
bool GPSFixAccepted = 0;
bool startup = false;

// ISR's
void IRAM_ATTR onTimer0(){
  portENTER_CRITICAL_ISR(&timer0Mux);
  interrupt0++;
  portEXIT_CRITICAL_ISR(&timer0Mux);
}

void IRAM_ATTR onTimer1(){
  portENTER_CRITICAL_ISR(&timer1Mux);
  interrupt1++; 
  portEXIT_CRITICAL_ISR(&timer1Mux);
}

void IRAM_ATTR onTimer2(){
  portENTER_CRITICAL_ISR(&timer2Mux);
  interrupt2++; 
  portEXIT_CRITICAL_ISR(&timer2Mux);
}

// Make sensor and server objects
TinyGPSPlus gps;
TinyGPSCustom fix(gps, "GPGSA", 2);
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
WiFiClient serverClients[MAX_SRV_CLIENTS];
AsyncWebServer webServer(config.http_port);
DNSServer dnsServer;

static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do 
  {
    while (Serial2.available())
     gps.encode(Serial2.read());   
  } 
  while (millis() - start < ms);
}

void callbackT0(){
  /*
  if (!bYouRang){
    delay(200);
    Serial.println("");
    Serial.println(F("You rang Sir?"));
    bYouRang = 1;
  }
  */
}

void callbackT3(){
  
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("- failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void getJSandCSSFiles(fs::FS &fs, const char * dirname, uint8_t levels){
    int fileCounter = 0;
    String fileName = "";
    Serial.printf("Listing directory: %s\r\n", dirname);
    File root = fs.open(dirname);
    if(!root){
        Serial.println("- failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return;
    }
    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            fileName = file.name();
            if(fileName.startsWith("main.")){
              if(!cssJsFileNamesConcat){
                if(fileName.endsWith(".js")){
                  fileJs.concat(file.name());
                  //Serial.println(fileJs);
                } else if (fileName.endsWith(".js.map")){
                  fileJsMap.concat(file.name());
                  //Serial.println(fileJsMap);
                } else if (fileName.endsWith(".css")){
                  fileCss.concat(file.name());
                  //Serial.println(fileCss);
                } else if (fileName.endsWith(".css.map")){
                  fileCssMap.concat(file.name());
                  //Serial.println(fileCssMap);
                }
              }
              Serial.print(file.name());
            }else {
              Serial.print(file.name());
            }
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
    cssJsFileNamesConcat = true;
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    File root = fs.open(path);
    if((!root) || (!root.isDirectory())) {
        if(fs.mkdir(path)){
            Serial.println("Dir created");
        } else {
            Serial.println("mkdir failed");
        }
        return;
    }
}

void printFile(fs::FS &fs, const char * path) {
  File file = fs.open(path);
  if (!file) {
    Serial.println(F("Failed to read file"));
    return;
  }
  while (file.available()) {
    Serial.print((char)file.read());
  }
  Serial.println();
  file.close();
}

String readFile(fs::FS &fs, const char * path){
  File file = fs.open(path);
  if(!file || file.isDirectory()){
    Serial.println("- empty file or failed to open file");
    return String();
  }
  String fileContent;
  while(file.available()){
    fileContent+=String((char)file.read());
  }
  return fileContent;
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\r\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("- file renamed");
    } else {
        Serial.println("- rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

String humanReadableSize(const size_t bytes) {
    if (bytes < 1024) return String(bytes) + " B";
    else if (bytes < (1024 * 1024)) return String(bytes / 1024.0) + " KB";
    else if (bytes < (1024 * 1024 * 1024)) return String(bytes / 1024.0 / 1024.0) + " MB";
    else return String(bytes / 1024.0 / 1024.0 / 1024.0) + " GB";
}

void saveConfigDataToJSON(){
  configDoc["asAP"] = config.asAP;
  configDoc["clientSSID"] = config.clientSSID;
  configDoc["clientPasswd"] = config.clientPasswd;
  configDoc["connectionTimeOut"] = config.connectionTimeOut;
  configDoc["deviceName"] = config.deviceName;
  configDoc["apPasswd"] = config.apPasswd;
  configDoc["gpsPollSec"] = String(config.gpsPollSec);
  configDoc["targetReached"] = String(config.targetReached);  
  configDoc["compPollMs"] = String(config.compPollMs);
  configDoc["compOffset"] = String(config.compOffset);
  configDoc["declAngleRad"] = String(config.declAngleRad,14);
  configDoc["sleepMins"] = String(config.sleepMins);
  configDoc["touchThreshold"] = String(config.touchThreshold);
  configDoc["touchEnabled"] = config.touchEnabled;
  configDoc["maxDistance"] = String(config.maxDistance);
  configDoc["maxDelay"] = String(config.maxDelay);
  configDoc["timeZoneOffset"] = String(config.timeZoneOffset);
}

// Saves the configuration to a file
void saveConfiguration(fs::FS &fs, const char * path, const Config &config) {
  deleteFile(LittleFS, path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to open file for writing."));
    return;
  }
  saveConfigDataToJSON();
  if (serializeJson(configDoc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }else{
    Serial.println(F("New file written"));
  }
  file.close();
}

void putJSONConfigDataInMemory(){
  config.asAP = configDoc["asAP"].as<bool>();
  String tempClientSSID = configDoc["clientSSID"];
  tempClientSSID.toCharArray(config.clientSSID, 16);
  String tempClientPasswd = configDoc["clientPasswd"];
  tempClientPasswd.toCharArray(config.clientPasswd, 16);
  config.connectionTimeOut = configDoc["connectionTimeOut"].as<int>();
  String tempDeviceName = configDoc["deviceName"];
  tempDeviceName.toCharArray(config.deviceName, 16);
  String tempApPasswd = configDoc["apPasswd"];
  tempApPasswd.toCharArray(config.apPasswd, 16);
  config.gpsPollSec = configDoc["gpsPollSec"].as<float>();
  config.targetReached = configDoc["targetReached"].as<int>();
  config.compPollMs = configDoc["compPollMs"].as<float>();
  config.compOffset = configDoc["compOffset"].as<float>();
  config.declAngleRad = configDoc["declAngleRad"].as<double>();
  config.sleepMins = configDoc["sleepMins"].as<int>();
  config.touchThreshold = configDoc["touchThreshold"].as<int>();  
  config.touchEnabled = configDoc["touchEnabled"].as<bool>();  
  config.maxDistance = configDoc["maxDistance"].as<int>();
  config.maxDelay = configDoc["maxDelay"].as<int>();
  config.timeZoneOffset = configDoc["timeZoneOffset"].as<int>();
  config.ftpEnabled = configDoc["ftpEnabled"].as<bool>();
}

void IRAM_ATTR loadConfiguration(fs::FS &fs, const char *path, Config config) {
  Serial.println(F("Loading configuration..."));
  Serial.println(path);
  File file = fs.open(path, FILE_READ);
  delay(100);
  DeserializationError error = deserializeJson(configDoc, file);
  if (error){
    Serial.println(F("Failed to read file, using default configuration"));
    Serial.println(error.c_str());
    saveConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str(), config);
  }else{
  putJSONConfigDataInMemory();
  file.close();
  }
}

  void saveCalibrationDataToJSON(){
  calDataDoc["magBiasX"] = String(caldata.magBiasX,6);
  calDataDoc["magBiasY"] = String(caldata.magBiasY,6);
  calDataDoc["magBiasZ"] = String(caldata.magBiasZ,6);  
  calDataDoc["magScaleFacX"] = String(caldata.magScaleFacX,6);
  calDataDoc["magScaleFacY"] = String(caldata.magScaleFacY,6);
  calDataDoc["magScaleFacZ"] = String(caldata.magScaleFacZ,6);  
  calDataDoc["gyroBiasX"] = String(caldata.gyroBiasX,6);
  calDataDoc["gyroBiasY"] = String(caldata.gyroBiasY,6);
  calDataDoc["gyroBiasZ"] = String(caldata.gyroBiasZ,6);
  calDataDoc["accelBiasX"] = String(caldata.accelBiasX,6);
  calDataDoc["accelBiasY"] = String(caldata.accelBiasY,6);
  calDataDoc["accelBiasZ"] = String(caldata.accelBiasZ,6);
  calDataDoc["accelScaleX"] = String(caldata.accelScaleX,6);
  calDataDoc["accelScaleY"] = String(caldata.accelScaleY,6);
  calDataDoc["accelScaleZ"] = String(caldata.accelScaleZ,6);
  }

// Saves the configuration to a file
void saveCalibrationData(fs::FS &fs, const char * path, const CalData &caldata) {
  deleteFile(LittleFS, path);
  Serial.println(path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
    return;
  }
  saveCalibrationDataToJSON();
  if (serializeJson(calDataDoc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }else{
    Serial.println(F("New file written"));
  }
  file.close();
}

void putJSONCalibrationDataInMemory() {
  caldata.magBiasX = calDataDoc["magBiasX"].as<float>();
  caldata.magBiasY = calDataDoc["magBiasY"].as<float>();
  caldata.magBiasZ = calDataDoc["magBiasZ"].as<float>();  
  caldata.magScaleFacX = calDataDoc["magScaleFacX"].as<float>();
  caldata.magScaleFacY = calDataDoc["magScaleFacY"].as<float>();
  caldata.magScaleFacZ = calDataDoc["magScaleFacZ"].as<float>();    
  caldata.gyroBiasX = calDataDoc["gyroBiasX"].as<float>();
  caldata.gyroBiasY = calDataDoc["gyroBiasY"].as<float>();
  caldata.gyroBiasZ = calDataDoc["gyroBiasZ"].as<float>();
  caldata.accelBiasX = calDataDoc["accelBiasX"].as<float>();
  caldata.accelBiasY = calDataDoc["accelBiasY"].as<float>();
  caldata.accelBiasZ = calDataDoc["accelBiasZ"].as<float>();    
  caldata.accelScaleX = calDataDoc["accelScaleX"].as<float>();    
  caldata.accelScaleY = calDataDoc["accelScaleY"].as<float>();    
  caldata.accelScaleZ = calDataDoc["accelScaleZ"].as<float>();  
}

// Loads the calibration data from a file
void IRAM_ATTR loadCalibrationData(fs::FS &fs, const char * path, CalData &caldata) {
  Serial.println(F("Loading calibration data..."));
  File file = fs.open(path, FILE_READ);
  delay(10);
  DeserializationError error = deserializeJson(calDataDoc, file);

  if (error){
    Serial.println(F("Failed to read file, using default calibration data."));
    Serial.println(error.c_str());
    saveCalibrationData(LittleFS, path, caldata);
  }else{
  putJSONCalibrationDataInMemory();
  file.close();
  }
}

 void saveDebugSettingsToJSON() {
  debugSettingsDoc["debugHaptic"] = debugSettings.debugHaptic;
  debugSettingsDoc["debug2Serial"] = debugSettings.debug2Serial;
  debugSettingsDoc["debugGPS2Serial"] = debugSettings.debugGPS2Serial;  
}

// Saves the configuration to a file
void saveDebugSettings(fs::FS &fs, const char * path, const DebugSettings &debugSettings) {
  deleteFile(LittleFS, path);
  Serial.println(path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
    return;
  }
  saveDebugSettingsToJSON();
  if (serializeJson(debugSettingsDoc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }else{
    Serial.println(F("New file written"));
  }
  file.close();
}

void putJSONDebugSettingsInMemory() {
  debugSettings.debugHaptic = debugSettingsDoc["debugHaptic"].as<bool>();  
  debugSettings.debug2Serial = debugSettingsDoc["debug2Serial"].as<bool>();
  debugSettings.debugGPS2Serial = debugSettingsDoc["debugGPS2Serial"].as<bool>(); 
}

// Loads the debug settings from a file
void IRAM_ATTR loadDebugSettings(fs::FS &fs, const char * path, DebugSettings &debugSettings) {
  Serial.println(F("Loading debug settings..."));
  File file = fs.open(path, FILE_READ);
  delay(10);
  DeserializationError error = deserializeJson(debugSettingsDoc, file);

  if (error){
    Serial.println(F("Failed to read file, using default debug settings."));
    Serial.println(error.c_str());
    saveDebugSettings(LittleFS, path, debugSettings);
  }else{
  putJSONDebugSettingsInMemory();
  file.close();
  }
}

void saveSensorDataToJSON() {
  sensorDataDoc["sensor"] = "gps";
  sensorDataDoc["time"] = GPSTimeMins;
  sensorDataDoc["ownLat"] = sensorData.ownLat;  
  sensorDataDoc["ownLon"] = sensorData.ownLon;
  sensorDataDoc["compassHeading"] = sensorData.compassHeading;
  sensorDataDoc["compassCardinal"] = sensorData.compassCardinal;
  sensorDataDoc["homeBaseLat"] = sensorData.homeBaseLat;
  sensorDataDoc["homeBaseLon"] = sensorData.homeBaseLon;
  sensorDataDoc["homeBaseBearing"] = sensorData.homeBaseBearing;
  sensorDataDoc["homeBaseCardinal"] = sensorData.homeBaseCardinal;
  sensorDataDoc["homeBaseDistance"] = sensorData.homeBaseDistance;
  sensorDataDoc["wayPointLat"] = sensorData.wayPointLat;
  sensorDataDoc["wayPointLon"] = sensorData.wayPointLon;
  sensorDataDoc["wayPointBearing"] = sensorData.wayPointBearing;
  sensorDataDoc["wayPointCardinal"] = sensorData.wayPointCardinal;
  sensorDataDoc["wayPointDistance"] = sensorData.wayPointDistance;
  sensorDataDoc["nrOfSatellites"] = sensorData.nrOfSatellites;
}

// Saves the GPS data to a file
void saveSensorData(fs::FS &fs, const char * path, const SensorData &sensorData) {
  deleteFile(LittleFS, path);
  Serial.println(path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
    return;
  }
  saveSensorDataToJSON();
  if (serializeJson(sensorDataDoc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }else{
    Serial.println(F("New file written"));
  }
  file.close();
}

void putJSONSensorDataInMemory() {
  String temp = sensorDataDoc["sensor"];
  temp.toCharArray(sensorData.sensorName, 8);  
  sensorData.gpsTime = sensorDataDoc["gpsTime"].as<int>();
  sensorData.ownLat = sensorDataDoc["ownLat"].as<double>(); 
  sensorData.ownLon = sensorDataDoc["ownLon"].as<double>();
  sensorData.compassHeading = sensorDataDoc["compassHeading"].as<float>();
  String temp3 = sensorDataDoc["compassCardinal"];
  temp3.toCharArray(sensorData.compassCardinal, 8); 
  sensorData.homeBaseLat = sensorDataDoc["homeBaseLat"].as<double>();
  sensorData.homeBaseLon = sensorDataDoc["homeBaseLon"].as<double>();
  sensorData.homeBaseBearing = sensorDataDoc["homeBaseBearing"].as<float>();
  String temp1 = sensorDataDoc["homebaseCardinal"];
  temp1.toCharArray(sensorData.homeBaseCardinal, 8);
  sensorData.homeBaseDistance = sensorDataDoc["homeBaseDistance"].as<int>();
  sensorData.wayPointLat = sensorDataDoc["wayPointLat"].as<double>(); 
  sensorData.wayPointLon = sensorDataDoc["wayPointLon"].as<double>();  
  sensorData.wayPointBearing = sensorDataDoc["wayPointBearing"].as<float>();
  String temp2 = sensorDataDoc["wayPointCardinal"];
  temp2.toCharArray(sensorData.wayPointCardinal, 8);
  sensorData.wayPointDistance = sensorDataDoc["wayPointDistance"].as<int>();  
  sensorData.nrOfSatellites = sensorDataDoc["nrOfSatellites"].as<int>();
}

// Loads the debug settings from a file
void IRAM_ATTR loadSensorData(fs::FS &fs, const char * path, SensorData &sensorData) {
  Serial.println(F("Loading sensor data..."));
  File file = fs.open(path, FILE_READ);
  delay(10);
  DeserializationError error = deserializeJson(sensorDataDoc, file);

  if (error){
    Serial.println(F("Failed to read file, using default sensor data."));
    Serial.println(error.c_str());
    saveSensorData(LittleFS, path, sensorData);
  }else{
  putJSONSensorDataInMemory();
  file.close();
  }
}

// Read info from json
// StaticJsonDocument<768> doc;
// DeserializationError error = deserializeJson(doc, input);

// if (error) {
//   Serial.print("deserializeJson() failed: ");
//   Serial.println(error.c_str());
//   return;
// }

// for (JsonObject map : doc["maps"].as<JsonArray>()) {

//   int map_id = map["id"]; // 1, 2, 2
//   const char* map_name = map["name"]; // "Test area", "Borderwar 12", "Borderwar 13"
//   const char* map_area = map["area"]; // "Kaag en Braassem", "BW12", "BW13"
//   const char* map_country = map["country"]; // "Netherlands", "Czech Republic", "Czech Republic"
//   const char* map_pngFile = map["pngFile"]; // "kaagenbraassem.png", "bw12.png", "bw13.png"
//   const char* map_kmlFile = map["kmlFile"]; // "kaagenbraassem.kml", "bw12.kml", "bw13.kml"
// }

// Put info in json
// StaticJsonDocument<768> doc;
// JsonArray maps = doc.createNestedArray("maps");

// JsonObject maps_0 = maps.createNestedObject();
// maps_0["id"] = 1;
// maps_0["name"] = "Test area";
// maps_0["area"] = "Kaag en Braassem";
// maps_0["country"] = "Netherlands";
// maps_0["pngFile"] = "kaagenbraassem.png";
// maps_0["kmlFile"] = "kaagenbraassem.kml";

// JsonObject maps_1 = maps.createNestedObject();
// maps_1["id"] = 2;
// maps_1["name"] = "Borderwar 12";
// maps_1["area"] = "BW12";
// maps_1["country"] = "Czech Republic";
// maps_1["pngFile"] = "bw12.png";
// maps_1["kmlFile"] = "bw12.kml";

// JsonObject maps_2 = maps.createNestedObject();
// maps_2["id"] = 3;
// maps_2["name"] = "Borderwar 13";
// maps_2["area"] = "BW13";
// maps_2["country"] = "Czech Republic";
// maps_2["pngFile"] = "bw13.png";
// maps_2["kmlFile"] = "bw13.kml";

// JsonObject maps_3 = maps.createNestedObject();
// maps_3["id"] = 4;
// maps_3["name"] = "Borderwar 14";
// maps_3["area"] = "BW14";
// maps_3["country"] = "Czech Republic";
// maps_3["pngFile"] = "bw14.png";
// maps_3["kmlFile"] = "bw14.kml";

// JsonObject maps_4 = maps.createNestedObject();
// maps_4["id"] = 5;
// maps_4["name"] = "Borderwar 15";
// maps_4["area"] = "BW15";
// maps_4["country"] = "Czech Republic";
// maps_4["pngFile"] = "bw15.png";
// maps_4["kmlFile"] = "bw15.kml";

// JsonObject maps_5 = maps.createNestedObject();
// maps_5["id"] = 6;
// maps_5["name"] = "Borderwar 16";
// maps_5["area"] = "BW16";
// maps_5["country"] = "Czech Republic";
// maps_5["pngFile"] = "bw16.png";
// maps_5["kmlFile"] = "bw16.kml";

// JsonObject maps_6 = maps.createNestedObject();
// maps_6["id"] = 7;
// maps_6["name"] = "Borderwar 17";
// maps_6["area"] = "BW17";
// maps_6["country"] = "Czech Republic";
// maps_6["pngFile"] = "bw17.png";
// maps_6["kmlFile"] = "bw17.kml";

// JsonObject maps_7 = maps.createNestedObject();
// maps_7["id"] = 8;
// maps_7["name"] = "Borderwar 18";
// maps_7["area"] = "BW18";
// maps_7["country"] = "Czech Republic";
// maps_7["pngFile"] = "bw18.png";
// maps_7["kmlFile"] = "bw18.kml";

// serializeJson(doc, output);

int findFreeSpotForMap(){
  int chosenMap = 1;
  return chosenMap;
}

void setMap1(String name, String area, String country, String pngFile, String kmlFile){
JsonObject maps_1 = maps.createNestedObject();
maps_1["id"] = 1;
maps_1["name"] = name;
maps_1["area"] = area;
maps_1["country"] = country;
maps_1["pngFile"] = mapsDir + pngFile;
maps_1["kmlFile"] = mapsDir + kmlFile;
}

void setMap2(String name, String area, String country, String pngFile, String kmlFile){
JsonObject maps_2 = maps.createNestedObject();
maps_2["id"] = 2;
maps_2["name"] = name;
maps_2["area"] = area;
maps_2["country"] = country;
maps_2["pngFile"] = mapsDir + pngFile;
maps_2["kmlFile"] = mapsDir + kmlFile;
}

void setMap3(String name, String area, String country, String pngFile, String kmlFile){
JsonObject maps_3 = maps.createNestedObject();
maps_3["id"] = 3;
maps_3["name"] = name;
maps_3["area"] = area;
maps_3["country"] = country;
maps_3["pngFile"] = mapsDir + pngFile;
maps_3["kmlFile"] = mapsDir + kmlFile;
}

void setMap4(String name, String area, String country, String pngFile, String kmlFile){
JsonObject maps_4 = maps.createNestedObject();
maps_4["id"] = 4;
maps_4["name"] = name;
maps_4["area"] = area;
maps_4["country"] = country;
maps_4["pngFile"] = mapsDir + pngFile;
maps_4["kmlFile"] = mapsDir + kmlFile;
}

void setMap5(String name, String area, String country, String pngFile, String kmlFile){
JsonObject maps_5 = maps.createNestedObject();
maps_5["id"] = 5;
maps_5["name"] = name;
maps_5["area"] = area;
maps_5["country"] = country;
maps_5["pngFile"] = mapsDir + pngFile;
maps_5["kmlFile"] = mapsDir + kmlFile;
}

void setMap6(String name, String area, String country, String pngFile, String kmlFile){
JsonObject maps_6 = maps.createNestedObject();
maps_6["id"] = 6;
maps_6["name"] = name;
maps_6["area"] = area;
maps_6["country"] = country;
maps_6["pngFile"] = mapsDir + pngFile;
maps_6["kmlFile"] = mapsDir + kmlFile;
}

void setMap7(String name, String area, String country, String pngFile, String kmlFile){
JsonObject maps_7 = maps.createNestedObject();
maps_7["id"] = 7;
maps_7["name"] = name;
maps_7["area"] = area;
maps_7["country"] = country;
maps_7["pngFile"] = mapsDir + pngFile;
maps_7["kmlFile"] = mapsDir + kmlFile;
}

void setMap8(String name, String area, String country, String pngFile, String kmlFile){
JsonObject maps_8 = maps.createNestedObject();
maps_8["id"] = 8;
maps_8["name"] = name;
maps_8["area"] = area;
maps_8["country"] = country;
maps_8["pngFile"] = mapsDir + pngFile;
maps_8["kmlFile"] = mapsDir + kmlFile;
}

void saveMapInPosition(int mapSelector, String name, String area, String country, String PNGFile, String KMLFile){

  switch(mapSelector){
    case 1:
      setMap1(name, area, country, PNGFile, KMLFile);
      break;
    case 2:
      setMap2(name, area, country, PNGFile, KMLFile);
      break;
    case 3:
      setMap3(name, area, country, PNGFile, KMLFile);
      break;
    case 4:
      setMap4(name, area, country, PNGFile, KMLFile);
      break;
    case 5:
      setMap5(name, area, country, PNGFile, KMLFile);
      break;
    case 6:
      setMap6(name, area, country, PNGFile, KMLFile);
      break;
    case 7:
      setMap7(name, area, country, PNGFile, KMLFile);
      break;
    case 8:
      setMap8(name, area, country, PNGFile, KMLFile);
      break;            
  }
}

String prepMapNameForMapDB(String fileName){
  String response = "";
    if(fileName.endsWith(".png")){
      uploadedPNGFile = fileName;
    }else if (fileName.endsWith(".kml")){
      uploadedKMLFile = fileName;
      Serial.println(uploadedPNGFile + " uploaded.");
      Serial.println(uploadedKMLFile + " uploaded.");
      response = "Files uploaded.";
    }else{
      Serial.println("Wrong type of file.");
      deleteFile(LittleFS, fileName.c_str());
      response = "Wrong filetype.";
    }
    return response; 
}

void addMaptoDB(String PNGFile, String KMLFile, JsonObject obj){
  int id = obj["id"];
  String name = obj["name"];
  String area = obj["area"];
  String country = obj["country"];
  PNGFile = mapsDir + "/" + PNGFile;
  KMLFile = mapsDir + "/" + KMLFile;
  mapSelector = id;
  saveMapInPosition(mapSelector, name, area, country, PNGFile, KMLFile);
  // save JSON DOC to file!!!
  Serial.println(name);
  Serial.println(area);
  Serial.println(country);
  Serial.println(PNGFile);
  Serial.println(KMLFile);
};

void removeMapFromDB(String filenName, int id){
    Serial.println("Removing map from DB.");
    deleteFile(LittleFS, filenName.c_str());
}

// Map saving and loading
 void saveSelectedMapToJSON() {

}

void saveSelectedMap(fs::FS &fs, const char * path, const SelectedMap &selectedMap) {
  deleteFile(LittleFS, path);
  Serial.println(path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
    return;
  }
  saveSelectedMapToJSON();
  if (serializeJson(mapListDoc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }else{
    Serial.println(F("New file written"));
  }
  file.close();
}

void putJSONSelectedMapInMemory() {

}

void IRAM_ATTR loadMapList(fs::FS &fs, const char * path, SelectedMap &selectedMap) {
  Serial.println(F("Loading debug settings..."));
  File file = fs.open(path, FILE_READ);
  delay(10);
  DeserializationError error = deserializeJson(mapListDoc, file);

  if (error){
    Serial.println(F("Failed to read file, using default debug settings."));
    Serial.println(error.c_str());
    saveSelectedMap(LittleFS, path, selectedMap);
  }else{
  putJSONSelectedMapInMemory();
  file.close();
  }
}

void readMapsFromJSON(fs::FS &fs, const char * path){
  Serial.println("Reading maps from mapData.json");
  File file = fs.open(path, FILE_READ);
  DeserializationError error = deserializeJson(mapListDoc, file);

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  for (JsonObject map : mapListDoc["maps"].as<JsonArray>()) {
    int map_id = map["id"]; // 1, 2, 2
    const char* map_name = map["name"];
    const char* map_area = map["area"];
    const char* map_country = map["country"];
    const char* map_pngFile = map["pngFile"];
    const char* map_kmlFile = map["kmlFile"];
  }
}

String getGPSTimeMinsSecs(){
  hours = gps.time.hour() + config.timeZoneOffset;
  mins = gps.time.minute();
  secs = gps.time.second();
  if(hours<10){
    GPSTimeMinsSecs = "0" + String(hours);  
  }else{
    if(hours > 23){
      GPSTimeMinsSecs = "00";
    }else {
      GPSTimeMinsSecs = String(hours);  
    }
  }
  if(mins<10){
    GPSTimeMinsSecs = GPSTimeMinsSecs + ":" + "0" + String(mins);
  }else{
    GPSTimeMinsSecs = GPSTimeMinsSecs + ":" + String(mins);
    }
  if(secs<10){
    GPSTimeMinsSecs = GPSTimeMinsSecs + ":" + "0" + String(secs);      
  }else{
    GPSTimeMinsSecs = GPSTimeMinsSecs + ":" + String(secs);
  }
  return GPSTimeMinsSecs;
}

String getGPSTimeMins(){
  hours = gps.time.hour() + config.timeZoneOffset;
  mins = gps.time.minute();
  if(hours<10){
    GPSTimeMins = "0" + String(hours);
  }else{
    if(hours > 23){
      GPSTimeMins = "00";
    }else {
      GPSTimeMins = String(hours);
    }
  }
  if(mins<10){
    GPSTimeMins = GPSTimeMins + ":" + "0" + String(mins);
  }else{
    GPSTimeMins = GPSTimeMins + ":" + String(mins);
    }
  return GPSTimeMins;
}

String getGPSDate(){
  day = gps.date.day();
  month = gps.date.month();
  year = gps.date.year();
  GPSDate = String(year) + "-" + String(month) + "-" + String(day);
  return GPSDate;
}

float GetCompassHeading(){
  float sum = 0.0;
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat magValue = myMPU9250.getMagValues();
  float temp = myMPU9250.getTemperature();
  float resultantG = myMPU9250.getResultantG(gValue);
  previous_compassheading = compassheading;
  // Calculate heading
  float heading = atan2(magValue.y, magValue.x);
  //float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = config.declAngleRad;
  heading += declinationAngle;
  
  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0){
    heading += 2.0 * PI;
  }

  if (heading > 2.0 * PI){
    heading -= 2.0 * PI;
  }
  // Convert to degrees
  float headingDegrees = heading * 180.0/M_PI;
  headingraw = headingDegrees;

    if(config.compOffset >= 0){
      if(headingDegrees < config.compOffset){
        headingDegrees = headingDegrees - config.compOffset + 360.0;
      }else{
        headingDegrees = headingDegrees - config.compOffset;
      }
    }else{
        headingDegrees = headingDegrees + (-1.0 * config.compOffset);
        if(headingDegrees >= 360)
        {
            headingDegrees = headingDegrees - 360;
        }
    }
  return headingDegrees;
}

unsigned long distance2waypoint(float waypoint_latt, float waypoint_long){ 
  unsigned long distanceToWaypoint = (unsigned long)TinyGPSPlus::distanceBetween(sensorData.ownLat,sensorData.ownLon,waypoint_latt,waypoint_long);
  return distanceToWaypoint;
}

float coarse2waypoint(float waypoint_latt, float waypoint_long){
  float coarseToWaypoint = TinyGPSPlus::courseTo(sensorData.ownLat,sensorData.ownLon,waypoint_latt,waypoint_long);
  return coarseToWaypoint;
}

float CalcRelHeading(float compforheading,float coarseforWaypoint){
  float relativeHeading;
  if (coarseforWaypoint > compforheading){   
    relativeHeading = coarseforWaypoint - compforheading;
  }
  else{
    relativeHeading = 360.0 - compforheading + coarseforWaypoint;
  }
  return relativeHeading;
}

void updateSensorData(){
  if (millis() > 5000 && gps.charsProcessed() < 10){
      Serial.println(F("No GPS data received: check wiring"));
  } else {
    smartDelay(50);
    String temp;
    sensorData.gpsTime = gps.time.value();
    sensorData.ownLat = gps.location.lat();
    sensorData.ownLon = gps.location.lng();
    sensorData.compassHeading = GetCompassHeading();
    temp = TinyGPSPlus::cardinal(sensorData.compassHeading);
    temp.toCharArray(sensorData.compassCardinal,8);
    sensorData.relheadingHomeBase = CalcRelHeading(compassheading, sensorData.homeBaseBearing);
    sensorData.homeBaseBearing = coarse2waypoint(sensorData.ownLat, sensorData.ownLon);
    sensorData.homeBaseDistance = distance2waypoint(sensorData.homeBaseLat, sensorData.homeBaseLon);
    temp = TinyGPSPlus::cardinal(sensorData.homeBaseBearing);
    temp.toCharArray(sensorData.homeBaseCardinal,8);
    sensorData.wayPointBearing = coarse2waypoint(sensorData.wayPointLat, sensorData.wayPointLon);
    sensorData.wayPointDistance = distance2waypoint(sensorData.wayPointLat, sensorData.wayPointLon);      
    sensorData.relheading = CalcRelHeading(compassheading, sensorData.wayPointBearing);
    temp = TinyGPSPlus::cardinal(sensorData.wayPointBearing); 
    temp.toCharArray(sensorData.wayPointCardinal,8);
    sensorData.nrOfSatellites = gps.satellites.value();
    getGPSTimeMins();
    saveSensorDataToJSON();
  }

  if (nrsatt > 3){
    digitalWrite(led,HIGH);
  }else{
    digitalWrite(led,LOW);
  }

  if (fix.isUpdated()){
    GPSFix = atol(fix.value());         
  }
}

void getInitialReadings(){
  smartDelay(50);
  String temp;
  sensorData.ownLat = gps.location.lat();
  sensorData.ownLon = gps.location.lng();
  sensorData.compassHeading = GetCompassHeading();
  temp = TinyGPSPlus::cardinal(sensorData.compassHeading);
  temp.toCharArray(sensorData.compassCardinal,8);
  previous_compassheading = sensorData.compassHeading;
  sensorData.homeBaseBearing = coarse2waypoint(sensorData.homeBaseLat, sensorData.homeBaseLon);
  sensorData.homeBaseDistance = distance2waypoint(sensorData.homeBaseLat, sensorData.homeBaseLon);
  temp = TinyGPSPlus::cardinal(sensorData.homeBaseBearing);
  temp.toCharArray(sensorData.homeBaseCardinal,8);
  sensorData.wayPointBearing = coarse2waypoint(sensorData.wayPointLat, sensorData.wayPointLon);
  sensorData.wayPointDistance = distance2waypoint(sensorData.wayPointLat, sensorData.wayPointLat);
  sensorData.relheading = CalcRelHeading(sensorData.compassHeading,sensorData.wayPointBearing);
  temp = TinyGPSPlus::cardinal(sensorData.wayPointBearing);
  temp.toCharArray(sensorData.wayPointCardinal,8);
  sensorData.nrOfSatellites = gps.satellites.value();
  getGPSTimeMins();
  getGPSDate();
  saveSensorDataToJSON();
}

// Replaces placeholder with value
String processor(const String& var){
  if(var == "JSFILE"){
        str2HTML = fileJs;
        return str2HTML;
    }else if (var == "CSSFILE"){
        str2HTML = fileCss;
        return str2HTML; 
    }else if (var == "HOSTADDRESS"){
        str2HTML = hostAddress;
        return str2HTML; 
    }
  return String();
}

void handleUpload(AsyncWebServerRequest *request, String filenameLocal, size_t index, uint8_t *data, size_t len, bool final){
  String response = "";
  // Serial.println("handling upload");
  if (!index) {
    String fileForDeletion = mapsDir + "/" + filenameLocal;
    LittleFS.rename(fileForDeletion.c_str(), "/maps/temp.png");
    request->_tempFile = LittleFS.open(mapsDir + "/" + filenameLocal, "w");
  }
  if (len) {
    request->_tempFile.write(data, len);
  }
  if (final) {
    request->_tempFile.close();
    response = prepMapNameForMapDB(filenameLocal);
    request->send(200, "application/json", response );
    LittleFS.remove("/maps/temp.png");
    listDir(LittleFS, mapsDir.c_str(), 0);
  }
}

void webServerSetup(){
// Webserver setup responses
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, "/index.html", String(), false, processor);
      timerRestart(timer2);
    }
  );

  webServer.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      saveConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str(), config);
      esp_restart();
    }
  );

  webServer.onRequestBody(
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
        {
            if ((request->url() == "/settings/settings_form") && (request->method() == HTTP_POST))
            {
                if (DeserializationError::Ok == deserializeJson(configDoc, (const char*)data))
                {
                    JsonObject obj = configDoc.as<JsonObject>();
                    putJSONConfigDataInMemory();
                    saveConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str(), config);
                    if(config.asAP){
                      delay(1000);
                      esp_restart();
                    }
                }
                request->send(200, "application/json", "{ \"status\": 0 }");
            } else if ((request->url() == "/settings/calibration_form") && (request->method() == HTTP_POST))
            {
                if (DeserializationError::Ok == deserializeJson(calDataDoc, (const char*)data))
                {
                    JsonObject obj = calDataDoc.as<JsonObject>();
                    putJSONCalibrationDataInMemory();
                    saveCalibrationData(LittleFS, (jsonDir + fileCalDataJSON).c_str(), caldata);
                }
                request->send(200, "application/json", "{ \"status\": 0 }");
            } else if ((request->url() == "/settings/debug_form") && (request->method() == HTTP_POST))
            {
                if (DeserializationError::Ok == deserializeJson(debugSettingsDoc, (const char*)data))
                {
                    JsonObject obj = debugSettingsDoc.as<JsonObject>();
                    putJSONDebugSettingsInMemory();
                    saveDebugSettings(LittleFS, (jsonDir + fileDebugJSON).c_str(), debugSettings);
                }
                request->send(200, "application/json", "{ \"status\": 0 }");
            } else if ((request->url() == "/navigation/register-map") && (request->method() == HTTP_POST))
            {
                // /navigation/register-map"
                if (DeserializationError::Ok == deserializeJson(mapListDoc, (const char*)data))
                {
                    JsonObject obj = mapListDoc.as<JsonObject>();
                    addMaptoDB(uploadedPNGFile, uploadedKMLFile, obj);
                    // saveSelectedMap(LittleFS, (jsonDir + fileMapDataJSON).c_str(), selectedMap);
                    //mapSelector = mapSelector + 1;
                }
                request->send(200, "application/json", "{ \"status\": 0 }");
            } else if ((request->url() == "/navigation/request-map") && (request->method() == HTTP_POST))
            {
                // /navigation/request-map"
                if (DeserializationError::Ok == deserializeJson(selectedMapJSON, (const char*)data))
                {
                    //JsonObject obj = mapListDoc.as<JsonObject>();
                    putJSONSelectedMapInMemory();
                    //saveSelectedMap(LittleFS, (jsonDir + filename_mapData).c_str(), selectedMap);
                }
                request->send(200, "application/json", "{ \"status\": 0 }");
            }     
        }
    );

  webServer.on(fileCss.c_str(), HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, fileCss.c_str(), "text/css");
    }
  );

  webServer.on(fileCssMap.c_str(), HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, fileCssMap.c_str(), "text/css");
    }
  );

  webServer.on(fileJs.c_str(), HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, fileJs.c_str(), "application/javascript");
    }
  );

  webServer.on(fileJsMap.c_str(), HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, fileJsMap.c_str(), "application/javascript");
    }
  );

  webServer.on("/listFiles", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      listDir(LittleFS, mapsDir.c_str(), 0);
      request->send(200, "application/json", "{ \"status\": 0 }");
    }
  );

  webServer.on("/getGPSTimeDate", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(200, "application/json", "{ \"GPSTime\" : \"" + GPSTimeMins + "\"" + "," + "\"GPSDate\" : \"" + getGPSDate() + "\"" + "}");
    }
  );

  webServer.on("/getGPSTime", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(200, "application/json", "{ \"GPSTime\" : \"" + GPSTimeMins + "\"" + "}");
    }
  );

  webServer.on("/getGPSDate", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(200, "application/json", "{ \"GPSDate\" : \"" + GPSDate + "\" }");
    }
  );

  webServer.on("/getDeviceName", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      String deviceName = config.deviceName;
      request->send(200, "application/json", "{ \"deviceName\" : \"" + deviceName + "\" }");
    }
  );

  webServer.on("/getSensorData", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      String temp;
      serializeJson(sensorDataDoc, temp);
      request->send(200, "application/json", temp );
    }
  );

  webServer.on("/setHome", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      sensorData.homeBaseLat = gps.location.lat();
      sensorData.homeBaseLon = gps.location.lng();
      saveSensorData(LittleFS, (jsonDir + fileSensorDataJSON).c_str(), sensorData);
      request->send(200, "application/json", "{ \"status\": 0 }");
    }
  );

  webServer.on(fileConfigJSON.c_str(), HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, (jsonDir + fileConfigJSON).c_str(), "application/json");
    }
  );

  webServer.on(fileCalDataJSON.c_str(), HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, (jsonDir + fileCalDataJSON).c_str(), "application/json");
    }
  );

  webServer.on(fileDebugJSON.c_str(), HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, (jsonDir + fileDebugJSON).c_str(), "application/json");
    }
  );

  webServer.on(fileMapDataJSON.c_str(), HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, (jsonDir + fileMapDataJSON).c_str(), "application/json");
    }
  );

  webServer.on("/maps/Home.png", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, "/maps/Home.png", "image/jpeg");
    }
  );

  webServer.on("/maps/NoMap.png", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, "/maps/NoMap.png", "image/jpeg");
    }
  );

  webServer.on("/manifest.json", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, "/manifest.json", "application/json");  
    }
  );

  webServer.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, "/favicon.ico", "image/ico");  
    }
  );

  webServer.on("/logo192.png", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, "/logo192.png", "image/png");  
    }
  );

  webServer.on("/deleteFile", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      deleteFile(LittleFS, "/maps/BW12.png");
      request->send(200, "application/json", "{ \"status\": 0 }"); 
    }
  );

  webServer.on(requestedMap.c_str(), HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, requestedMap.c_str(), "image/png");  
    }
  );

  webServer.on(selectedMapPNG.c_str(), HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, selectedMapPNG.c_str(), "image/png");  
    }
  );

  webServer.on("/upload-file", HTTP_POST, [](AsyncWebServerRequest * request) {
    request->send(200);
  }, handleUpload);

  webServer.onFileUpload(handleUpload);

  webServer.onNotFound( []( AsyncWebServerRequest * request )
    {
    request->send(LittleFS, "/redirect.html", String(), false, processor);
    timerRestart(timer2);
    Serial.println("redirect called");
  });
  
  webServer.begin();
  Serial.print("HTTP Started on port: ");
  Serial.println(config.http_port);
}

void timerSetup(){
  // timer0 setup
  timer0 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer0, &onTimer0, true);
  timerAlarmWrite(timer0, (config.gpsPollSec * 1000000), true);           // 1000 ms
// timer1 setup
  timer1 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, (config.compPollMs * 1000), true);            // 1 ms
// timer2 setup
  timer2 = timerBegin(2, 80, true);
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, (config.sleepMins * 60000000), true);            // 1 min
}

void PWMSetup(){
//PWM setup
  ledcSetup(pwmhapticfront, hapticfreq, resolution);
  ledcSetup(pwmhapticright, hapticfreq, resolution);
  ledcSetup(pwmhapticrear, hapticfreq, resolution);
  ledcSetup(pwmhapticleft, hapticfreq, resolution);
  ledcAttachPin(hapticfront, pwmhapticfront);
  ledcAttachPin(hapticright, pwmhapticright);
  ledcAttachPin(hapticrear, pwmhapticrear);
  ledcAttachPin(hapticleft, pwmhapticleft);
  digitalWrite(led, 0);
}

void wifiSetup(){
      delay(500);
    // Connect to Wi-Fi network with SSID and password
  if (config.asAP) {
      delay(1000);
      WiFi.mode( WIFI_MODE_APSTA );
      IPAddress ip( 192, 168, 1, 1 );
      IPAddress gateway( 192, 168, 1, 1 );
      IPAddress subnet( 255, 255, 255, 0 );
      WiFi.softAP(config.deviceName,config.apPasswd);
      delay(2000);
      WiFi.softAPConfig( ip, gateway, subnet );
      Serial.print("Setting HaptiCap up as AP: ");
      Serial.println(config.deviceName);
      Serial.println(config.apPasswd);      
      IPAddress IP = WiFi.softAPIP();
      Serial.print("AP IP address: ");
      Serial.println(IP);
  }else{
      Serial.print("Setting HapiCap as client to network ");
      Serial.print(config.clientSSID);
      WiFi.mode(WIFI_STA);
      WiFi.begin(config.clientSSID, config.clientPasswd);
      intCounterWifi = 0;
      
    while (WiFi.status() != WL_CONNECTED){
      delay(500);
      Serial.print(".");
      intCounterWifi++;
        if (intCounterWifi > 120){
          Serial.println("");
          config.asAP = 1;
          saveConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str(), config);
          delay(1000);
          ESP.restart();          
        }
      }

  // initialize WiFi
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(config.clientSSID);
    IPAddress IP = WiFi.localIP();
    ipAddress = IP.toString();
    hostAddress = "http://" + ipAddress;
  }
}

void magnometerSetup(){
    Serial.println("");
    byte whoAmICode = 0x00;
  Wire.begin();
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }

  if(!myMPU9250.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
  }
  else{
    Serial.println("Magnetometer is connected");
  }

  myMPU9250.autoOffsets();
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);  // lowest noise
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
}

void haptiCapReady(){
  Serial.print("HaptiCap ready @ ");
  Serial.println(getGPSTimeMinsSecs() + " " + getGPSDate());
  Serial.print("Satellites: ");
  Serial.println(sensorData.nrOfSatellites);
  startup = true;
}

void setup(){
  // Serial port for debugging purposes
  Serial.begin(SerialUSBBaud);
  Serial2.begin(GPSBaud);
  delay(1000);

// Initialize LittleFS
if (!LittleFS.begin(false /* false: Do not format if mount failed */)) {
  Serial.println("Failed to mount LittleFS");
  if (!LittleFS.begin(true /* true: format */)) {
    Serial.println("Failed to format LittleFS");
  } else {
    Serial.println("LittleFS formatted successfully");
  }
} else { 
  getJSandCSSFiles(LittleFS, "/", 0);
}

  Serial.println();
  delay(1000);
  loadConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str(), config);
  loadDebugSettings(LittleFS, (jsonDir + fileDebugJSON).c_str(), debugSettings);  
  
  Serial.println();
  Serial.print("HaptiCap version: ");
  Serial.println(SWVERSION);
  Serial.println("By Chrysnet.com and Triznet.com");

//Setup interrupt on Touch Pad 1 (GPIO0) and wake up
  touchAttachInterrupt(T0, callbackT0, config.touchThreshold);
  touchAttachInterrupt(T3, callbackT3, config.touchThreshold);
  esp_sleep_enable_touchpad_wakeup();
  
// Initialize outputs
  pinMode(buttonPin,INPUT);
  pinMode(led, OUTPUT);
  pinMode(dta_rdy_pin,INPUT);

  timerSetup();
  PWMSetup();
  wifiSetup();
  magnometerSetup();
  
  loadCalibrationData(LittleFS, (jsonDir + fileCalDataJSON).c_str(), caldata);
  delay(1000);
  getInitialReadings();
  timerAlarmEnable(timer0);
  //timerAlarmEnable(timer1);
  //timerAlarmEnable(timer2);

  timerStart(timer0);
  //timerStart(timer1);
  //timerStart(timer2);  

  if(!config.asAP){
    Serial.print("IP address: ");
    Serial.println(ipAddress);
    Serial.print("Host address: ");
    Serial.println(hostAddress);
    }else{
      dnsServer.start(config.dns_port, "*", WiFi.softAPIP());
      Serial.print("DNS Started on port.");
      Serial.println(config.dns_port);
    }

  webServerSetup();
  delay(1000);
  loadSensorData(LittleFS, (jsonDir + fileSensorDataJSON).c_str(), sensorData);
  //saveMapInPosition(1, "Home", "kaag en Braassem", "Netherlands", "Home.png", "Home.kml");
  readMapsFromJSON(LittleFS, (jsonDir + fileMapDataJSON).c_str());
  haptiCapReady();
}
 
void loop(){
  if(config.asAP){  
    dnsServer.processNextRequest();
  }

// Start of main code.
  if(timers_disabled){
    timers_disabled = 0; 
    timerStart(timer0);
    timerStart(timer1);
    timerStart(timer2);                
  }

  if(interrupt0 > 0){
    portENTER_CRITICAL(&timer0Mux);
    interrupt0--;    
    portEXIT_CRITICAL(&timer0Mux);
      updateSensorData();
    }

  if(interrupt1 > 0){
    portENTER_CRITICAL(&timer1Mux);
    interrupt1--;      
    portEXIT_CRITICAL(&timer1Mux);
    sensorData.compassHeading = GetCompassHeading();
    if(interrupt1 > 10){
      interrupt1 = 2;
    }                 
  }

  // if(interrupt2){
  //   portENTER_CRITICAL(&timer2Mux);
  //   interrupt2--;
  //   portEXIT_CRITICAL(&timer2Mux);
  //   Serial.println(F("Going to sleep... ZZZZZZZZZZZzzzzzzzzzzZZZZZZZZZzzzzzzzzzz"));
  //   bYouRang = 0;
  //   interrupt0 = 0;
  //   interrupt1 = 0;
  //   interrupt2 = 0;
  //   esp_deep_sleep_start();          
  // }
}
