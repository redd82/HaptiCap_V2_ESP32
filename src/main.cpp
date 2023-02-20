#include <Arduino.h>

#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include <DNSServer.h>
#include "SPIFFS.h"
#include "TinyGPS++.h"
#include <ESP8266FtpServer.h>
#include <ArduinoJson.h>
#include <MPU9250_WE.h>
#include <Wire.h>
#include <string>
#include "functions.h"

#define SWVERSION 2.001
#define MPU9250_ADDR 0x68
#define MAX_SRV_CLIENTS 2
#define INPUT_SIZE 20
#define CONFIG_JSON_DOCSIZE 1024
#define CALDATA_JSON_DOCSIZE 768
#define DEBUGSETTINGS_JSON_DOCSIZE 768
#define GPSDATA_JSON_DOCSIZE 1024

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
  bool debug2Telnet = false;
  bool debug2Serial = false;
  bool debugData2Serial = false;
};

struct GPSData {
  char sensorName[8];
  int gpsTime;
  float ownLat;
  float ownLon;
  float homeBaseLat;
  float homeBaseLon;
  float homeBaseBearing;
  String homeBaseDirection;
  float homeBaseDistance;
  float wayPointBearing;
  String wayPointDirection;
  float compassHeading;
  String compassDirection;
  int nrOfSatellites;
};

Config config;                         // <- global configuration object
CalData caldata;
DebugSettings debugSettings;
GPSData gpsData;

// _PARAMS_
const char* filename = "/hapticap.json";
const char* filename_cal = "/caldata.json";
const char* filename_debug = "/debug.json";
String fileJs = "/";
String fileJsMap = "/";
String fileCss = "/";
String fileCssMap = "/";
static const uint32_t GPSBaud = 9600;
static const uint32_t SerialUSBBaud = 115200;
static int taskCore = 0;
const char* cardinalCompHeading;
const char* cardinalToWaypoint;
const char* cardinal2home;

// _Interrupts_
int interrupt0;
int interrupt1;
int interrupt2;
int totalInterruptCounter;

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

RTC_DATA_ATTR float flCurrentLat;
RTC_DATA_ATTR float flCurrentLon;
RTC_DATA_ATTR bool bDumpConfig = 1;
RTC_DATA_ATTR bool bTargetReachedAck = 0;
RTC_DATA_ATTR bool bHomeReachedAck = 0;

String inputLat = String(config.WAYPOINT_LAT,6);
String inputLon = String(config.WAYPOINT_LON,6);
String inputSetting = "false";

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
float coarsewaypoint = 0;
unsigned long distancewaypoint = 0;
float relheading = 0;
unsigned long distance2home = 0;
float relheading2home = 0;
float coarse2home = 0;
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
String proficiency;
bool name_received = false;
bool proficiency_received = false;
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

// #include "webserver_processor.h"
// #include "config_save_load.h"

// Make sensor and server objects
StaticJsonDocument<CONFIG_JSON_DOCSIZE> configDoc;
StaticJsonDocument<CALDATA_JSON_DOCSIZE> calDataDoc;
StaticJsonDocument<DEBUGSETTINGS_JSON_DOCSIZE> debugSettingsDoc;
DynamicJsonDocument GPSDataDoc(GPSDATA_JSON_DOCSIZE);

TinyGPSPlus gps;
TinyGPSCustom fix(gps, "GPGSA", 2);
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
WiFiServer TelNetserver(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];
AsyncWebServer webServer(config.http_port);
DNSServer dnsServer;
FtpServer ftpSrv;

/*
if (magneticVariation.isUpdated())
  {
    Serial.print("Magnetic variation is ");
    Serial.println(magneticVariation.value());
  }
*/

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

void getInitialReadings(){
  // compassheading = GetCompassHeading();
  // cardinalCompHeading = TinyGPSPlus::cardinal(compassheading);
  // previous_compassheading = compassheading;
  // coarsewaypoint = coarse2waypoint(config.WAYPOINT_LAT, config.WAYPOINT_LON);
  // distancewaypoint = distance2waypoint(config.WAYPOINT_LAT, config.WAYPOINT_LON);
  // relheading = CalcRelHeading(compassheading,coarsewaypoint);
  // cardinalToWaypoint = TinyGPSPlus::cardinal(coarsewaypoint);
  // coarse2home = coarse2waypoint(config.HOME_LAT, config.HOME_LON);
  // distance2home = distance2waypoint(config.HOME_LAT, config.HOME_LON);
  // cardinal2home = TinyGPSPlus::cardinal(coarse2home);
  flCurrentLat = gps.location.lat();
  flCurrentLon = gps.location.lng();
  nrsatt = gps.satellites.value();
}

void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
  if(!index){
    Serial.printf("UploadStart: %s\n", filename.c_str());
  }
  for(size_t i=0; i<len; i++){
    Serial.write(data[i]);
  }
  if(final){
    Serial.printf("UploadEnd: %s, %u B\n", filename.c_str(), index+len);
  }
}

// Replaces placeholder with value
void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

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

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
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
                if(fileName.endsWith(".js")){
                  fileJs.concat(file.name());
                  Serial.print(fileJs);
                } else if (fileName.endsWith(".js.map")){
                  fileJsMap.concat(file.name());
                  Serial.print(fileJsMap);
                } else if (fileName.endsWith(".css")){
                  fileCss.concat(file.name());
                  Serial.print(fileCss);
                } else if (fileName.endsWith(".css.map")){
                  fileCssMap.concat(file.name());
                  Serial.print(fileCssMap);
                }
            }else {
              Serial.print(file.name());
            }
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void printFile(fs::FS &fs, const char * path) {
  // Open file for reading
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
  //Serial.println("- read from file:");
  String fileContent;
  while(file.available()){
    fileContent+=String((char)file.read());
  }
  //Serial.println(fileContent);
  return fileContent;
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
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
  configDoc["HOME_LAT"] = String(config.HOME_LAT,6);
  configDoc["HOME_LON"] = String(config.HOME_LON,6);
  configDoc["WAYPOINT_LAT"] = String(config.WAYPOINT_LAT,6);
  configDoc["WAYPOINT_LON"] = String(config.WAYPOINT_LON,6);  
  configDoc["declAngleRad"] = String(config.declAngleRad,14);
  configDoc["sleepMins"] = String(config.sleepMins);
  configDoc["touchThreshold"] = String(config.touchThreshold);
  configDoc["touchEnabled"] = config.touchEnabled;
  configDoc["maxDistance"] = String(config.maxDistance);
  configDoc["maxDelay"] = String(config.maxDelay);
  configDoc["timeZoneOffset"] = String(config.timeZoneOffset);
  configDoc["ftpEnabled"] = config.ftpEnabled;
}

// Saves the configuration to a file
void saveConfiguration(fs::FS &fs, const char * path, const Config &config) {
  deleteFile(SPIFFS, path);
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
  config.HOME_LAT = configDoc["HOME_LAT"].as<float>();
  config.HOME_LON = configDoc["HOME_LON"].as<float>();
  config.WAYPOINT_LAT = configDoc["WAYPOINT_LAT"].as<float>();
  config.WAYPOINT_LON = configDoc["WAYPOINT_LON"].as<float>();  
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
    saveConfiguration(SPIFFS, filename, config);
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
  deleteFile(SPIFFS, path);
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
    saveCalibrationData(SPIFFS, path, caldata);
  }else{
  putJSONCalibrationDataInMemory();
  file.close();
  }
}

 void saveDebugSettingsToJSON() {
  debugSettingsDoc["debugHaptic"] = debugSettings.debugHaptic;
  debugSettingsDoc["debug2Serial"] = debugSettings.debug2Serial;
  debugSettingsDoc["debugData2Serial"] = debugSettings.debugData2Serial;  
  debugSettingsDoc["debug2Telnet"] = debugSettings.debug2Telnet;
}

// Saves the configuration to a file
void saveDebugSettings(fs::FS &fs, const char * path, const DebugSettings &debugSettings) {
  deleteFile(SPIFFS, path);
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
  debugSettings.debugHaptic = debugSettingsDoc["debugHaptic"].as<int>();  
  debugSettings.debug2Serial = debugSettingsDoc["debug2Serial"].as<int>();
  debugSettings.debugData2Serial = debugSettingsDoc["debugData2Serial"].as<int>(); 
  debugSettings.debug2Telnet = debugSettingsDoc["debug2Telnet"].as<int>(); 
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
    saveDebugSettings(SPIFFS, path, debugSettings);
  }else{
  putJSONDebugSettingsInMemory();
  file.close();
  }
}

  // char sensorName[8];
  // int gpsTime;
  // float ownLat;
  // float ownLon;
  // float homeBaseLat;
  // float homeBaseLon;
  // float homeBaseBearing;
  // String homeBaseDirection;
  // float homeBaseDistance;
  // float wayPointBearing;
  // String wayPointDirection;
  // float compassHeading;
  // String compassDirection;
  // int nrOfSatellites;

 void saveGPSDataToJSON() {
  GPSDataDoc["sensor"] = "gps";
  GPSDataDoc["time"] = gpsData.gpsTime;
  GPSDataDoc["ownLat"] = gpsData.ownLat;  
  GPSDataDoc["ownLon"] = gpsData.ownLon;
  GPSDataDoc["homeBaseLat"] = gpsData.homeBaseLat;
  GPSDataDoc["homeBaseLon"] = gpsData.homeBaseLon;
  GPSDataDoc["homeBaseBearing"] = gpsData.homeBaseBearing;
  GPSDataDoc["homeBaseDirection"] = gpsData.homeBaseDirection;
  GPSDataDoc["homeBaseDistance"] = gpsData.homeBaseDistance;
  GPSDataDoc["wayPointBearing"] = gpsData.wayPointBearing;
  GPSDataDoc["wayPointDirection"] = gpsData.wayPointDirection;
  GPSDataDoc["compassHeading"] = gpsData.compassHeading;
  GPSDataDoc["compassDirection"] = gpsData.compassDirection;
  GPSDataDoc["nrOfSatellites"] = gpsData.nrOfSatellites;
}

// Saves the GPS data to a file
void saveGPSData(fs::FS &fs, const char * path, const GPSData &gpsData) {
  deleteFile(SPIFFS, path);
  Serial.println(path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
    return;
  }
  saveGPSDataToJSON();
  if (serializeJson(GPSDataDoc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }else{
    Serial.println(F("New file written"));
  }
  file.close();
}

void putJSONGPSDataInMemory() {
  String tempSensorName = GPSDataDoc["sensor"];
  tempSensorName.toCharArray(gpsData.sensorName, 8);  
  gpsData.gpsTime = GPSDataDoc["gpsTime"].as<int>();
  gpsData.ownLat = GPSDataDoc["ownLat"].as<float>(); 
  gpsData.ownLon = GPSDataDoc["ownLon"].as<float>(); 
}

// Loads the debug settings from a file
void IRAM_ATTR loadGPSData(fs::FS &fs, const char * path, GPSData &gpsData) {
  Serial.println(F("Loading debug settings..."));
  File file = fs.open(path, FILE_READ);
  delay(10);
  DeserializationError error = deserializeJson(GPSDataDoc, file);

  if (error){
    Serial.println(F("Failed to read file, using default debug settings."));
    Serial.println(error.c_str());
    saveGPSData(SPIFFS, path, gpsData);
  }else{
  putJSONGPSDataInMemory();
  file.close();
  }
}

void printConfigInMemory(){
  Serial.print("As Accesspoint: ");
  Serial.println(config.asAP);
  Serial.print("Client to SSID: ");
  Serial.println(config.clientSSID);
  Serial.print("Client Password: ");
  Serial.println(config.clientPasswd);
  Serial.print("Connection Timeout: ");
  Serial.println(config.connectionTimeOut);
  Serial.print("Devicename: ");
  Serial.println(config.deviceName);
  Serial.print("Device password: ");
  Serial.println(config.apPasswd);
  Serial.print("GPS polling(s): ");
  Serial.println(config.gpsPollSec);
  Serial.print("Target reached: ");
  Serial.println(config.targetReached);
  Serial.print("Compass polling(ms): ");
  Serial.println(config.compPollMs);
  Serial.print("Compass offset(deg): ");
  Serial.println(config.compOffset);
  Serial.print("Homebase(lat): ");
  Serial.println(config.HOME_LAT,6);
  Serial.print("Homebase(lon):" );
  Serial.println(config.HOME_LON,6);
  Serial.print("Waypoint(lat): ");
  Serial.println(config.WAYPOINT_LAT,6);
  Serial.print("Waypoint(lon):" );
  Serial.println(config.WAYPOINT_LON,6);  
  Serial.print("Declanation angle(rads): ");
  Serial.println(config.declAngleRad,12);
  Serial.print("Sleeptime(mins): ");
  Serial.println(config.sleepMins);
  Serial.print("Threshold touch: ");
  Serial.println(config.touchThreshold);
  Serial.print("Enable touch: ");
  Serial.println(config.touchEnabled);
  Serial.print("Timezone offset: ");
  Serial.println(config.timeZoneOffset);
  Serial.print("Pulsed max distance: ");
  Serial.println(config.maxDistance);
  Serial.print("Max Delay between pulses: ");
  Serial.println(config.maxDelay);

  Serial.print("FTP Enabled: ");
  Serial.println(config.ftpEnabled);  
  bDumpConfig = 0;
  Serial.println(); 
}

void printCalDataInMemory(){
  Serial.print("magBiasX: ");
  Serial.println(caldata.magBiasX,6);
  Serial.print("magBiasY: ");
  Serial.println(caldata.magBiasY,6);
  Serial.print("magBiasZ: ");
  Serial.println(caldata.magBiasZ,6);
  Serial.print("magScaleFacX: ");
  Serial.println(caldata.magScaleFacX,6);
  Serial.print("magScaleFacY: ");
  Serial.println(caldata.magScaleFacY,6);
  Serial.print("magScaleFacZ: ");
  Serial.println(caldata.magScaleFacZ,6);
  Serial.print("gyroBiasX: ");
  Serial.println(caldata.gyroBiasX,6);
  Serial.print("gyroBiasY: ");
  Serial.println(caldata.gyroBiasY,6);
  Serial.print("gyroBiasZ: ");
  Serial.println(caldata.gyroBiasZ,6);
  Serial.print("accelBiasX: ");
  Serial.println(caldata.accelBiasX,6);
  Serial.print("accelBiasY: ");
  Serial.println(caldata.accelBiasY,6);
  Serial.print("accelBiasZ:" );
  Serial.println(caldata.accelBiasZ,6);
  Serial.print("accelScaleX: ");
  Serial.println(caldata.accelScaleX,6);
  Serial.print("accelScaleY:" );
  Serial.println(caldata.accelScaleY,6);  
  Serial.print("accelScaleZ: ");
  Serial.println(caldata.accelScaleZ,6);
  bDumpConfig = 0;
  Serial.println(); 
}

void printDebugSettingsInMemory(){
  Serial.print("Debug2Serial: ");
  Serial.println(debugSettings.debug2Serial);  
  Serial.print("Debug RAW GPS to telnet: ");
  Serial.println(debugSettings.debugData2Serial);
  Serial.print("Haptic feedback Debug: ");
  Serial.println(debugSettings.debugHaptic);
}

String getGPSTimeMinsSecs(){
  if(!startup){
    while(!gps.time.isUpdated()){
      //Serial.println(".");
    }
  }
    if (gps.time.isUpdated()){
        hours = gps.time.hour() + config.timeZoneOffset;
        mins = gps.time.minute();
        secs = gps.time.second();
      if(hours<10){
        GPSTimeMinsSecs = "0" + String(hours);  
      }else{
         GPSTimeMinsSecs = String(hours);  
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
      }
      return GPSTimeMinsSecs;
}

String getGPSTimeMins(){
    if (gps.time.isUpdated()){
        hours = gps.time.hour() + config.timeZoneOffset;
        mins = gps.time.minute();
      if(hours<10){
        GPSTimeMins = "0" + String(hours);  
      }else{
         GPSTimeMins = String(hours);  
        }
      if(mins<10){
        GPSTimeMins = GPSTimeMins + ":" + "0" + String(mins);
      }else{
        GPSTimeMins = GPSTimeMins + ":" + String(mins);
        }
      }
      return GPSTimeMins;
}

String getGPSDate(){
  if(!startup){
    while(!gps.date.isUpdated()){
      //Serial.println(".");
    }
  }
      if (gps.date.isUpdated()){
        day = gps.date.day();
        month = gps.date.month();
        year = gps.date.year();
        GPSDate = String(year) + "-" + String(month) + "-" + String(day);
      }
      return GPSDate;
}

unsigned long distance2waypoint(float waypoint_latt, float waypoint_long){ 
    unsigned long distanceToWaypoint = (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),waypoint_latt,waypoint_long);
    return distanceToWaypoint;
}

float coarse2waypoint(float waypoint_latt, float waypoint_long){
    float coarseToWaypoint = TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),waypoint_latt,waypoint_long);
    //const char *cardinalToWaypoint = TinyGPSPlus::cardinal(coarseToWaypoint);
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

// Printline functions
static void printFloat(float val, bool valid, int len, int prec){
  if (!valid){
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else{
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len){
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t){
  if (!d.isValid()){
    Serial.print(F("********** "));
  }
  else{
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid()){
    Serial.print(F("******** "));
  }
  else{
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }
  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len){
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}

static void printLines(unsigned long distance2Waypoint, float course2Waypoint, float compheading,const char *cardinal2Waypoint,float relheading,float relheading2h){
  if(bPrintHeader){
  Serial.print("TinyGPS Library version: ");
  Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Dist.   Course Card    Heading    Chars Sentences  Chksm   RelHDGWP  RelHDG2H   PWM      PWM      PWM     PWM     Touch0     Touch3  "));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  --- to Waypoint ---  --Compass--  RX    RX         Fail     (deg)      (deg)    Front    Right    Rear    Left                       "));
  Serial.println(F("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------"));
  }
  bPrintHeader = 0;
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 7);
  printInt(distance2Waypoint, gps.location.isValid(), 9);
  printFloat(course2Waypoint, gps.location.isValid(), 7, 2);
  printStr(gps.location.isValid() ? cardinal2Waypoint : "*** ", 5);
  printFloat((compheading),true,7,2);
  printStr(true ? TinyGPSPlus::cardinal(compheading) : "*** ", 6);
  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  printFloat((relheading),true,11,2);
  printFloat((relheading2h),true,11,2);          
  printInt(pwm_front, true, 9);
  printInt(pwm_right, true, 9);
  printInt(pwm_rear, true, 9);
  printInt(pwm_left, true, 9);
  printInt(touchValueT0, true, 9);
  printInt(touchValueT3, true, 9);  
  Serial.println();  
}

void webServerSetup(){
// Webserver setup responses
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      if(!config.ftpEnabled){
      request->send(SPIFFS, "/index.html", String(), false, processor);
      timerRestart(timer2);
      if(debugSettings.debug2Serial){
        Serial.println("index called");
      }
      } else{
        request->send(SPIFFS, "/ftpmode.html", String(), false, processor);
      }

    }
  );

    webServer.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      saveConfiguration(SPIFFS, filename, config);
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
                    saveConfiguration(SPIFFS, filename, config);
                    printConfigInMemory();
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
                    saveCalibrationData(SPIFFS, filename_cal, caldata);
                }
                request->send(200, "application/json", "{ \"status\": 0 }");
            } else if ((request->url() == "/settings/debug_form") && (request->method() == HTTP_POST))
            {
                if (DeserializationError::Ok == deserializeJson(debugSettingsDoc, (const char*)data))
                {
                    JsonObject obj = debugSettingsDoc.as<JsonObject>();
                    putJSONDebugSettingsInMemory();
                    saveDebugSettings(SPIFFS, filename_debug, debugSettings);
                }
                request->send(200, "application/json", "{ \"status\": 0 }");
            }
        }
    );

  webServer.on(fileCss.c_str(), HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(SPIFFS, fileCss.c_str(), "text/css");
    }
  );

  webServer.on(fileCssMap.c_str(), HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(SPIFFS, fileCssMap.c_str(), "text/css");
    }
  );

  webServer.on(fileJs.c_str(), HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(SPIFFS, fileJs.c_str(), "application/javascript");
      //request->send(SPIFFS, fileJs.c_str(), String(), false, processor);
    }
  );

  webServer.on(fileJsMap.c_str(), HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(SPIFFS, fileJsMap.c_str(), "application/javascript");
      //request->send(SPIFFS, fileJsMap.c_str(), String(), false, processor);
    }
  );

  webServer.on("/getGPSTime", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      String temp = getGPSTimeMins();
      request->send(200, "application/json", "{ \"GPSTime\" : \"" + temp + "\" }");
    }
  );

  webServer.on("/getGPSDate", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      String temp = getGPSDate();
      request->send(200, "application/json", "{ \"GPSDate\" : \"" + temp + "\" }");
    }
  );

  webServer.on("/hapticap.json", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(SPIFFS, "/hapticap.json", "application/json");
    }
  );

  webServer.on("/caldata.json", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(SPIFFS, "/caldata.json", "application/json");
    }
  );

    webServer.on("/debug.json", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(SPIFFS, "/debug.json", "application/json");
    }
  );

  webServer.on("/getConfigToSerial", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      printConfigInMemory();
      request->send(200, "application/json", "{ \"status\": 0 }");
    }
  );

  webServer.on("/map1.jpg", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(SPIFFS, "/map1.jpg", "image/jpeg");
    }
  );

  webServer.on("/manifest.json", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(SPIFFS, "/manifest.json", "application/json");  
    }
  );

    webServer.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(SPIFFS, "/favicon.ico", "image/ico");  
    }
  );

    webServer.on("/logo192.png", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(SPIFFS, "/logo192.png", "image/png");  
    }
  );

  webServer.onNotFound( []( AsyncWebServerRequest * request )
     {
      request->send(SPIFFS, "/redirect.html", String(), false, processor);
      timerRestart(timer2);
      Serial.println("redirect called");
    });
  
 
  webServer.begin();
  Serial.print("HTTP Started on port: ");
  Serial.println(config.http_port);
}

void webServerSetupFTP(){
  // Webserver setup responses
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      if(!config.ftpEnabled){
      request->send(SPIFFS, "/index.html", String(), false, processor);
      timerRestart(timer2);
      if(debugSettings.debug2Serial){
        Serial.println("index called");
      }
      } else{
        request->send(SPIFFS, "/ftpmode.html", String(), false, processor);
      }
    }
  );

    webServer.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      saveConfiguration(SPIFFS, filename, config);
      delay(1000);
      esp_restart();
    }
  );
  webServer.begin();
  Serial.print("HTTP Started on port: ");
  Serial.println(config.http_port);
}

void setup(){
  // Serial port for debugging purposes
  Serial.begin(SerialUSBBaud);
  Serial2.begin(GPSBaud);
  delay(1000);

  // Initialize SPIFFS
  if(!SPIFFS.begin(true)){
      while (1)
        Serial.println("SPIFFS.begin() failed");
  } else {
     listDir(SPIFFS, "/", 0);
  }

  Serial.println();
  delay(1000);
  loadConfiguration(SPIFFS, filename, config);
  loadDebugSettings(SPIFFS, filename_debug, debugSettings);  
  
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
          config.asAP = 1;
          saveConfiguration(SPIFFS, filename, config);
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
 
  Serial.println("");
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

  // Start server
  if (SPIFFS.begin(true)) {

  }

  while (Serial2.available()){
    gps.encode(Serial2.read());  
  }

loadCalibrationData(SPIFFS, filename_cal, caldata);
delay(1000);
getInitialReadings();
timerAlarmEnable(timer0);
timerAlarmEnable(timer1);
timerAlarmEnable(timer2);

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

  if(config.ftpEnabled){
    Serial.println("FTP started.");
    //webServerSetupFTP();
    config.ftpEnabled = false;
    saveConfiguration(SPIFFS, filename, config);
    delay(1000);
    config.ftpEnabled = true;
    ftpSrv.begin(config.deviceName,config.apPasswd);
  }else{
    webServerSetup();
  }

delay(1000);
Serial.print("HaptiCap ready @ ");
GPSTimeMinsSecs = getGPSTimeMinsSecs();
GPSDate = getGPSDate();
Serial.println(GPSTimeMinsSecs + " " + GPSDate);
Serial.println();
startup = true;
}
 
void loop(){
  if(config.ftpEnabled){
     ftpSrv.handleFTP();
  } else {
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
        while (Serial2.available()){
          gps.encode(Serial2.read());
        }
        if (debugSettings.debug2Serial){
          Serial.println("Position updated.");
        }
      }








  }
}