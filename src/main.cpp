#include <Arduino.h>

#include "WiFi.h"
#include "ESPAsyncWebServer.h"
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
#define CONFIG_JSON_DOCSIZE 860
#define CALDATA_JSON_DOCSIZE 700

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
  bool asAP = 0;
  char clientSSID[16] = "TrizNet_AP2";
  char clientPasswd[16] = "T0sh7b49";
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
  bool touchEnabled = 1;
  bool debugHaptic = 0;
  bool debug2Telnet = 0;
  bool debug2Serial = 0;
  bool debugData2Serial = 0;
  bool ftpEnabled = 0;
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

Config config;                         // <- global configuration object
CalData caldata;

// _PARAMS_
const char* filename = "/hapticap.json";
const char* filename_cal = "/caldata.json";
String fileJs = "/";
String fileJsMap = "/";
String fileCss = "/";
String fileCssMap = "/";
static const uint32_t GPSBaud = 9600;
static const uint32_t SerialUSBBaud = 115200;
static int taskCore = 0;
const char* PARAM_LAT = "lat";
const char* PARAM_LON = "lon";
const char* PARAM_LATHOME = "lathome";
const char* PARAM_LONHOME = "lonhome";
const char* PARAM_GENERALSETNORTH = "setnorth";
const char* PARAM_DEBUGTELNET = "debugtelnet";
const char* PARAM_DEBUGSERIAL = "debugserial";
const char* PARAM_DEBUGDATASERIAL = "debugDataserial";
const char* PARAM_DEBUGHAPTIC = "debughaptic";
const char* PARAM_DEBUGAPPLY = "DebugApply";
const char* PARAM_APMODE = "ap_mode";
const char* PARAM_FTPMODE = "FTPMode";
const char* PARAM_SSID = "ssid";
const char* PARAM_CLIENTPASSWD = "client_passwd";
const char* PARAM_DEVICENAME = "devicename";
const char* PARAM_AP_PASSWD = "ap_password";
const char* PARAM_NETWORKAPPLY = "NetworkApply";
const char* PARAM_COMPASSOFFSET = "compass_offset";
const char* PARAM_COMPASSDECLANGLE = "comapss_declangle";
const char* PARAM_COMPASSPOLLTIME = "compass_polltime";
const char* PARAM_COMPASSAPPLY = "CompassApply";
const char* PARAM_GPS = "gps";
const char* PARAM_GPSPOLLTIME = "gps_polltime";
const char* PARAM_GPSTARGETREACHED = "gps_targetreached";
const char* PARAM_GPSAPPLY = "GPSApply";
const char* PARAM_ESP_SLEEPTIME = "esp_sleeptime";
const char* PARAM_TOUCH = "touch";
const char* PARAM_TOUCHTHRESHOLD = "touch_threshold";
const char* PARAM_MAXDELAY = "gps_max_delay";
const char* PARAM_MAXDISTANCE = "gps_max_distance";
const char* PARAM_GENERAL = "general";
const char* PARAM_GENERALAPPLY = "GeneralApply";
const char* PARAM_RESTARTESP = "restart";    
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
bool bPrintHeader = 0;
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

String GPSTime;
int hours;
int mins;
int secs;
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
TinyGPSPlus gps;
TinyGPSCustom fix(gps, "GPGSA", 2);
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
WiFiServer TelNetserver(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];
AsyncWebServer webServer(config.http_port);
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

// Saves the configuration to a file
void saveConfiguration(fs::FS &fs, const char * path, const Config &config) {
  deleteFile(SPIFFS, path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to open file for writing."));
    return;
  }
  StaticJsonDocument<CONFIG_JSON_DOCSIZE> doc;
  doc["asAP"] = String(config.asAP);
  doc["clientSSID"] = config.clientSSID;
  doc["clientPasswd"] = config.clientPasswd;
  doc["deviceName"] = config.deviceName;
  doc["apPasswd"] = config.apPasswd;
  doc["gpsPollSec"] = String(config.gpsPollSec);
  doc["targetReached"] = String(config.targetReached);  
  doc["compPollMs"] = String(config.compPollMs);
  doc["compOffset"] = String(config.compOffset);
  doc["HOME_LAT"] = String(config.HOME_LAT,6);
  doc["HOME_LON"] = String(config.HOME_LON,6);
  doc["WAYPOINT_LAT"] = String(config.WAYPOINT_LAT,6);
  doc["WAYPOINT_LON"] = String(config.WAYPOINT_LON,6);  
  doc["declAngleRad"] = String(config.declAngleRad,14);
  doc["sleepMins"] = String(config.sleepMins);
  doc["touchThreshold"] = String(config.touchThreshold);
  doc["touchEnabled"] = String(config.touchEnabled);
  doc["debugHaptic"] = String(config.debugHaptic);
  doc["maxDistance"] = String(config.maxDistance);
  doc["maxDelay"] = String(config.maxDelay);
  doc["timeZoneOffset"] = String(config.timeZoneOffset);
  doc["debug2Serial"] = String(config.debug2Serial);
  doc["debugData2Serial"] = String(config.debugData2Serial);  
  doc["debug2Telnet"] = String(config.debug2Telnet);
  doc["ftpEnabled"] = String(config.ftpEnabled);
  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }
  file.close();
}

void IRAM_ATTR loadConfiguration(fs::FS &fs, const char *path, Config config) {
  Serial.println(F("Loading configuration..."));
  Serial.println(path);
  File file = fs.open(path, FILE_READ);
  delay(100);
  StaticJsonDocument<CONFIG_JSON_DOCSIZE> doc;
  DeserializationError error = deserializeJson(doc, file);
  if (error){
    Serial.println(F("Failed to read file, using default configuration"));
    Serial.println(error.c_str());
    saveConfiguration(SPIFFS, filename, config);
  }else{

  config.asAP = doc["asAP"].as<int>();
  String tempClientSSID = doc["clientSSID"];
  tempClientSSID.toCharArray(config.clientSSID, 16);
  String tempClientPasswd = doc["clientPasswd"];
  tempClientPasswd.toCharArray(config.clientPasswd, 16);
  String tempDeviceName = doc["deviceName"];
  tempDeviceName.toCharArray(config.deviceName, 16);
  String tempApPasswd = doc["apPasswd"];
  tempApPasswd.toCharArray(config.apPasswd, 16);
  config.gpsPollSec = doc["gpsPollSec"].as<float>();
  config.targetReached = doc["gpsTargetReached"].as<int>();
  config.compPollMs = doc["compPollMs"].as<float>();
  config.compOffset = doc["compOffset"].as<float>();
  config.HOME_LAT = doc["HOME_LAT"].as<float>();
  config.HOME_LON = doc["HOME_LON"].as<float>();
  config.WAYPOINT_LAT = doc["WAYPOINT_LAT"].as<float>();
  config.WAYPOINT_LON = doc["WAYPOINT_LON"].as<float>();  
  config.declAngleRad = doc["declAngleRad"].as<double>();
  config.sleepMins = doc["sleepMins"].as<int>();
  config.touchThreshold = doc["touchThreshold"].as<int>();  
  config.touchEnabled = doc["touchEnabled"].as<int>();  
  config.debugHaptic = doc["debugHaptic"].as<int>();  
  config.maxDistance = doc["maxDistance"].as<int>();
  config.maxDelay = doc["maxDelay"].as<int>();
  config.timeZoneOffset = doc["timeZoneOffset"].as<int>();
  config.debug2Serial = doc["debug2Serial"].as<int>();
  config.debugData2Serial = doc["debugData2Serial"].as<int>(); 
  config.debug2Telnet = doc["debug2Telnet"].as<int>(); 
  config.ftpEnabled = doc["ftpEnabled"].as<int>();
  file.close();
  }
}

// Saves the configuration to a file
void saveCalibrationData(fs::FS &fs, const char * path, const CalData &caldata) {
  deleteFile(SPIFFS, path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
    return;
  }

  StaticJsonDocument<CALDATA_JSON_DOCSIZE> doc;
  doc["magBiasX"] = String(caldata.magBiasX,6);
  doc["magBiasY"] = String(caldata.magBiasY,6);
  doc["magBiasZ"] = String(caldata.magBiasZ,6);  
  doc["magScaleFacX"] = String(caldata.magScaleFacX,6);
  doc["magScaleFacY"] = String(caldata.magScaleFacY,6);
  doc["magScaleFacZ"] = String(caldata.magScaleFacZ,6);  
  doc["gyroBiasX"] = String(caldata.gyroBiasX,6);
  doc["gyroBiasY"] = String(caldata.gyroBiasY,6);
  doc["gyroBiasZ"] = String(caldata.gyroBiasZ,6);
  doc["accelBiasX"] = String(caldata.accelBiasX,6);
  doc["accelBiasY"] = String(caldata.accelBiasY,6);
  doc["accelBiasZ"] = String(caldata.accelBiasZ,6);
  doc["accelScaleX"] = String(caldata.accelScaleX,6);
  doc["accelScaleY"] = String(caldata.accelScaleY,6);
  doc["accelScaleZ"] = String(caldata.accelScaleZ,6);
  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }
  file.close();
}

// Loads the calibration data from a file
void IRAM_ATTR loadCalibrationData(fs::FS &fs, const char * path, CalData &caldata) {
  File file = fs.open(path, FILE_READ);
  delay(10);
  StaticJsonDocument<CALDATA_JSON_DOCSIZE> doc;
  DeserializationError error = deserializeJson(doc, file);

  if (error){
    Serial.println(F("Failed to read file, using default configuration"));
    Serial.println(error.c_str());
    saveCalibrationData(SPIFFS, filename_cal, caldata);
  }else{
  
  caldata.magBiasX = doc["magBiasX"].as<float>();
  caldata.magBiasY = doc["magBiasY"].as<float>();
  caldata.magBiasZ = doc["magBiasZ"].as<float>();  
  caldata.magScaleFacX = doc["magScaleFacX"].as<float>();
  caldata.magScaleFacY = doc["magScaleFacY"].as<float>();
  caldata.magScaleFacZ = doc["magScaleFacZ"].as<float>();    
  caldata.gyroBiasX = doc["gyroBiasX"].as<float>();
  caldata.gyroBiasY = doc["gyroBiasY"].as<float>();
  caldata.gyroBiasZ = doc["gyroBiasZ"].as<float>();
  caldata.accelBiasX = doc["accelBiasX"].as<float>();
  caldata.accelBiasY = doc["accelBiasY"].as<float>();
  caldata.accelBiasZ = doc["accelBiasZ"].as<float>();    
  caldata.accelScaleX = doc["accelScaleX"].as<float>();    
  caldata.accelScaleY = doc["accelScaleY"].as<float>();    
  caldata.accelScaleZ = doc["accelScaleZ"].as<float>();    
  file.close();
  }
}

String GetGPSTime(){
      if (gps.time.isUpdated()){
        hours = gps.time.hour() + config.timeZoneOffset;
        mins = gps.time.minute();
        secs = gps.time.second();
      if(hours<10){
        GPSTime = "0" + String(hours);  
      }else{
         GPSTime = String(hours);  
        }
      if(mins<10){
        GPSTime = GPSTime + ":" + "0" + String(mins);
      }else{
        GPSTime = GPSTime + ":" + String(mins);
        }
      if(secs<10){
        GPSTime = GPSTime + ":" + "0" + String(secs);      
      }else{
        GPSTime = GPSTime + ":" + String(secs);
        }
      }
      return GPSTime;
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


void setup(){
  // Serial port for debugging purposes
  Serial.begin(SerialUSBBaud);
  Serial2.begin(GPSBaud);
  delay(1000);

// EEPROM setup  
  // if (!EEPROM.begin(1000)){
  //   Serial.println("Failed to initialise EEPROM");
  //   Serial.println("Restarting...");
  //   delay(1000);
  //   ESP.restart();
  // }

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
      WiFi.mode( WIFI_AP );
      IPAddress ip( 192, 168, 1, 1 );
      IPAddress gateway( 192, 168, 1, 1 );
      IPAddress subnet( 255, 255, 0, 0 );
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

// Webserver setup responses
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(SPIFFS, "/index.html", String(), false, processor);
      timerRestart(timer2);
      if(config.debug2Serial){
        Serial.println("index called");
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
    }
  );

  webServer.on(fileJsMap.c_str(), HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(SPIFFS, fileJsMap.c_str(), "application/javascript");
    }
  );

  webServer.on("/hapticap.json", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(SPIFFS, "/hapticap.json", "application/json");
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

// Send a GET request to <ESP_IP>/get?input1=<inputMessage> set.html?lat=123.456&lon=78.90  setlatlong?lat=123.456&lon=78.90
webServer.on("/setlatlon", HTTP_GET, [] (AsyncWebServerRequest *request)
  {
    if (request->hasParam(PARAM_LAT)) {
      inputLat = request->getParam(PARAM_LAT)->value();
      inputLon = request->getParam(PARAM_LON)->value();
    }
    else {
      inputLat = "0.0";
      inputLon = "0.0";
    }
    flCurrentLat = gps.location.lat();
    flCurrentLon = gps.location.lng();
    config.WAYPOINT_LAT = inputLat.toFloat();
    config.WAYPOINT_LON = inputLon.toFloat();    
    distancewaypoint = distance2waypoint(config.WAYPOINT_LAT, config.WAYPOINT_LON);
    coarsewaypoint = coarse2waypoint(config.WAYPOINT_LAT, config.WAYPOINT_LON);
    cardinalToWaypoint = TinyGPSPlus::cardinal(coarsewaypoint);
    if(config.debug2Serial){
      Serial.println("WP Set");
      Serial.println(config.WAYPOINT_LAT,6);
      Serial.println(config.WAYPOINT_LON,6);
    }    
    saveConfiguration(SPIFFS, filename, config);    
    request->send(SPIFFS, "/areamap.html", String(), false);
    timerRestart(timer2);    
  }
);

webServer.on("/setlatlon_home", HTTP_GET, [] (AsyncWebServerRequest *request){
    flCurrentLat = gps.location.lat();
    flCurrentLon = gps.location.lng();
    config.HOME_LAT = flCurrentLat;
    config.HOME_LON = flCurrentLon;
    coarse2home = coarse2waypoint(config.HOME_LAT, config.HOME_LON);
    distance2home = distance2waypoint(config.HOME_LAT, config.HOME_LON);
    cardinal2home = TinyGPSPlus::cardinal(coarse2home);
    nrsatt = gps.satellites.value();
    if(config.debug2Serial){
      Serial.println("Home Set");
      Serial.println(config.HOME_LAT,6);
      Serial.println(config.HOME_LON,6);
    }
    saveConfiguration(SPIFFS, filename, config);
    request->send(SPIFFS, "/areamap.html", String(), false);
    timerRestart(timer2);    
  });    
  
webServer.on( "/", HTTP_POST, []( AsyncWebServerRequest * request )
  {
    if ( request->authenticate( config.http_username, config.http_password ) )
    {
      request->send( 200 );
    }
    else
    {
      request->requestAuthentication();
    }
  },
  []( AsyncWebServerRequest * request, String filename, size_t index, uint8_t *data, size_t len, bool final )
  {
    static bool   _authenticated;
    static time_t startTimer;

    if ( !index )
    {
      _authenticated = false;
      if ( request->authenticate(config.http_username, config.http_password) )
      {
        startTimer = millis();
        Serial.printf( "UPLOAD: Started to receive '%s'.\n", filename.c_str() );
        _authenticated = true;
      }
      else
      {
        Serial.println( "Unauthorized access." );
        return request->send( 401, "text/plain", "Not logged in." );
      }      
    }

    if ( _authenticated )
    {
      //Serial.printf( "%i bytes received.\n", index );
      //Store or do something with the data...
    }


    if ( final && _authenticated )
    {
      Serial.printf( "UPLOAD: Done. Received %.2f kBytes in %.2fs which is %i kB/s.\n", index / 1024.0, ( millis() - startTimer ) / 1000.0, index / ( millis() - startTimer ) );
    }
  });

  webServer.onNotFound( []( AsyncWebServerRequest * request )
  {
    Serial.printf("NOT_FOUND: ");
    if (request->method() == HTTP_GET)
      Serial.printf("GET");
    else if (request->method() == HTTP_POST)
      Serial.printf("POST");
    else if (request->method() == HTTP_DELETE)
      Serial.printf("DELETE");
    else if (request->method() == HTTP_PUT)
      Serial.printf("PUT");
    else if (request->method() == HTTP_PATCH)
      Serial.printf("PATCH");
    else if (request->method() == HTTP_HEAD)
      Serial.printf("HEAD");  
    else if (request->method() == HTTP_OPTIONS)
      Serial.printf("OPTIONS");
    else
      Serial.printf("UNKNOWN");
    Serial.printf(" http://%s%s\n", request->host().c_str(), request->url().c_str());
    request->send( 404, "text/plain", "Not found." );
  });

  webServer.onNotFound(notFound);
  webServer.begin();
  
  // Start server
  if (SPIFFS.begin(true)) {

  }

  while (Serial2.available()){
    gps.encode(Serial2.read());  
  }

getInitialReadings();

timerAlarmEnable(timer0);
timerAlarmEnable(timer1);
timerAlarmEnable(timer2);
Serial.print("HaptiCap ready @ ");
GPSTime = GetGPSTime();
Serial.println(GPSTime);
Serial.print("IP address: ");
Serial.println(WiFi.localIP());
}
 
void loop(){
  if(config.ftpEnabled)
     ftpSrv.handleFTP();

}