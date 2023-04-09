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
#define SWVERSION 2.01
#define MPU9250_ADDR 0x68
#define MAX_SRV_CLIENTS 2
#define INPUT_SIZE 20
#define CONFIG_JSON_DOCSIZE 1024
#define CALDATA_JSON_DOCSIZE 768
#define DEBUGSETTINGS_JSON_DOCSIZE 768
#define SENSORDATA_JSON_DOCSIZE 1024
#define MAPRECIEVED_JSON_DOCSIZE 768
#define MAPLIST_JSON_DOCSIZE 4096
#define WAYPOINTS_JSON_DOCSIZE 12288
#define NROFWAYPOINTS 30
#define NROFMAPS 8

#define LEDCOMMON 19
#define LED7 18
#define LED6 5
#define LED5 4
#define LED4 14
#define LED3 27
#define LED2 26
#define LED1 25
#define LED0 33 

// ----- Gyro
#define MPU9250_I2C_address 0x68                                        // I2C address for MPU9250 
#define MPU9250_I2C_master_enable 0x6A                                  // USER_CTRL[5] = I2C_MST_EN
#define MPU9250_Interface_bypass_mux_enable 0x37                        // INT_PIN_CFG[1]= BYPASS_EN
#define Frequency 125                                                   // 8mS sample interval 
#define Sensitivity 65.5                                                // Gyro sensitivity (see data sheet)
#define Sensor_to_deg 1/(Sensitivity*Frequency)                         // Convert sensor reading to degrees
#define Sensor_to_rad Sensor_to_deg*DEG_TO_RAD                          // Convert sensor reading to radians
#define Loop_time 1000000/Frequency                                     // Loop time (uS)

// ----- Magnetometer
#define AK8963_I2C_address 0x0C                                             // I2C address for AK8963
#define AK8963_cntrl_reg_1 0x0A                                             // CNTL[4]=#bits, CNTL[3:0]=mode
#define AK8963_status_reg_1 0x02                                            // ST1[0]=data ready
#define AK8963_data_ready_mask 0b00000001                                   // Data ready mask
#define AK8963_overflow_mask 0b00001000                                     // Magnetic sensor overflow mask
#define AK8963_data 0x03                                                    // Start address of XYZ data                                                                
#define AK8963_fuse_ROM 0x10                                                // X,Y,Z fuse ROM

long Loop_start;                                                        // Loop start time (uS)

int     Gyro_x,     Gyro_y,     Gyro_z;
long    Gyro_x_cal, Gyro_y_cal, Gyro_z_cal;
float   Gyro_pitch, Gyro_roll, Gyro_yaw;
float   Gyro_pitch_output, Gyro_roll_output;

// ----- Accelerometer
long    Accel_x,      Accel_y,      Accel_z,    Accel_total_vector;
float   Accel_pitch,  Accel_roll;

// ----- Compass heading
/*
  The magnetic declination for Lower Hutt, New Zealand is +22.5833 degrees
  Obtain your magnetic declination from http://www.magnetic-declination.com/
  Uncomment the declination code within the main loop() if you want True North.
*/
float   Declination = +22.5833;                                             //  Degrees ... replace this declination with yours
float   Heading;

int     Mag_x,                Mag_y,                Mag_z;                  // Raw magnetometer readings
float   Mag_x_dampened,       Mag_y_dampened,       Mag_z_dampened;
float   Mag_x_hor, Mag_y_hor;
float   Mag_pitch, Mag_roll;

// ----- Record compass offsets, scale factors, & ASA values
/*
   These values seldom change ... an occasional check is sufficient
   (1) Open your Arduino "Serial Monitor
   (2) Set "Record_data=true;" then upload & run program.
   (3) Replace the values below with the values that appear on the Serial Monitor.
   (4) Set "Record_data = false;" then upload & rerun program.
*/
// bool    Record_data = true;
// int     Mag_x_offset = 46,      Mag_y_offset = 190,     Mag_z_offset = -254;   // Hard-iron offsets
// float   Mag_x_scale = 1.01,     Mag_y_scale = 0.99,     Mag_z_scale = 1.00;    // Soft-iron scale factors
// float   ASAX = 1.17,            ASAY = 1.18,            ASAZ = 1.14;           // (A)sahi (S)ensitivity (A)djustment fuse ROM values.

void setup();
void setupIO();
void webServerFunctions();
void loop();

// ----- Flags
bool Gyro_synchronised = false;
bool Flag = false;

// ----- Debug
#define Switch 23                       // Connect an SPST switch between A0 and GND to enable/disable tilt stabilazation
long Loop_start_time;
long Debug_start_time;


// Set declination angle on your location and fix heading
// Formula: (deg + (minutes / 60.0)) / (180 / M_PI); (4.0 + (26.0 / 60.0)) / (180 / PI);
//float declinationAngle = (declAngleDeg + (declAngleMin / 60.0)) / (180.0 / PI);
// _Config_

struct Config {
  uint8_t http_port = 80;
  uint8_t dns_port = 53;
  bool asAP = false;
  char clientSSID[25] = "TrizNet_AP";
  char clientPasswd[16] = "S4pphi099#";
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
  int selectedMap = 1;
};

// _CalData_
struct CalData {
  bool calibrateMag = false;
  float compassOffset = 0.0;
  int magBiasX = 0;             // Mag_x_offset = 46,
  int magBiasY = 0;             // Mag_y_offset = 190,
  int magBiasZ = 0;             // Mag_z_offset = -254;
  float magScaleFacX = 1.0;     // Mag_x_scale = 1.01,  
  float magScaleFacY = 1.0;     // Mag_y_scale = 0.99,
  float magScaleFacZ = 1.0;     // Mag_z_scale = 1.00;
  float ASAX = 1.0;             // (A)sahi (S)ensitivity (A)djustment fuse ROM values.
  float ASAY = 1.0;
  float ASAZ = 1.0;
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

struct WaypointsMap {
  double homeBase[2] = { 0.0 };
  double wayPoint [NROFWAYPOINTS][2];
};

struct SelectedMap {
  // id, name, country, area, mapfile, kmlfile
  int map_id; // 1
  char map_name[32] = ""; // "Test area"
  char map_area[32] = ""; // "Kaag en Braassem"
  char map_country[64] = ""; // "Netherlands"
  char map_pngFile[32] = ""; // "kaagenbraassem.png"
  int imageWidth = 0;
  int imageHeight = 0;
  char map_kmlFile[16] = ""; // "kaagenbraassem.kml"
  double realWorldHeight = 0.0;
  double realWorldWidth = 0.0;
  float scaleHeight = 1.0;
  float scaleWidth = 1.0;
  double north = 0.0;
  double west = 0.0;
  double south = 0.0;
  double east = 0.0;
  float rotation = 0.0;
  int radius = 63713000;
};

StaticJsonDocument<CONFIG_JSON_DOCSIZE> configDoc;
StaticJsonDocument<CALDATA_JSON_DOCSIZE> calDataDoc;
StaticJsonDocument<DEBUGSETTINGS_JSON_DOCSIZE> debugSettingsDoc;
StaticJsonDocument<SENSORDATA_JSON_DOCSIZE> sensorDataDoc;
StaticJsonDocument <MAPLIST_JSON_DOCSIZE> mapListDoc;
StaticJsonDocument <MAPRECIEVED_JSON_DOCSIZE> selectedMapJSON;
StaticJsonDocument <WAYPOINTS_JSON_DOCSIZE> waypointsMapsDoc;
JsonArray maps = mapListDoc.createNestedArray("maps");
JsonArray mapWaypoints = waypointsMapsDoc.createNestedArray("mapWaypoints");

Config config;                         // <- global configuration object
CalData caldata;
DebugSettings debugSettings;
SensorData sensorData;
WaypointsMap wayPoints;
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
String requestedMap = "";
int mapSelector = 1;
static const uint32_t GPSBaud = 9600;
static const uint32_t SerialUSBBaud = 115200;
static int taskCore = 0;

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

// const int dta_rdy_pin = 19;
const int led = 23;
//const int buttonPin = 4;
const int TouchPinT0 = 4;
const int TouchPinT3 = 15;
int buttonState = 0;
touch_pad_t touchPin;
int touchValueT0;
int touchValueT3;

// PWM settings for haptic feedback and LED feedback
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

#define LEDCOMMON 19
#define LED7 18
#define LED6 5
#define LED5 4
#define LED4 14
#define LED3 27
#define LED2 26
#define LED1 25
#define LED0 33 

const int ledcommon = 19;
const int led0 = 33;
const int led1 = 25;
const int led2 = 26;
const int led3 = 27;
const int led4 = 14;
const int led5 = 4;
const int led6 = 5;
const int led7 = 18;

const int pwmled0 = 0;
const int pwmled1 = 1;
const int pwmled2 = 2;
const int pwmled3 = 3;
const int pwmled4 = 4;
const int pwmled5 = 5;
const int pwmled6 = 6;
const int pwmled7 = 7;

int pwm_front = 0;
int pwm_right = 0;
int pwm_rear = 0;
int pwm_left = 0;
bool bPrintHeader = false;
int count = 0;
int i = 0;

RTC_DATA_ATTR double flCurrentLat;
RTC_DATA_ATTR double flCurrentLon;
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
bool asAP = false;
bool startup = false;
bool enableLed0;
bool enableLed1;
bool enableLed2;
bool enableLed3;
bool enableLed4;
bool enableLed5;
bool enableLed6;
bool enableLed7;

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

// file management
String humanReadableSize(const size_t bytes) {
    if (bytes < 1024) return String(bytes) + " B";
    else if (bytes < (1024 * 1024)) return String(bytes / 1024.0) + " KB";
    else if (bytes < (1024 * 1024 * 1024)) return String(bytes / 1024.0 / 1024.0) + " MB";
    else return String(bytes / 1024.0 / 1024.0 / 1024.0) + " GB";
}

String printFreeSpace(){
  String freeSpace = "";
  Serial.print("Free: "); Serial.println(humanReadableSize((LittleFS.totalBytes() - LittleFS.usedBytes())));
  freeSpace = "Free: " + humanReadableSize((LittleFS.totalBytes() - LittleFS.usedBytes()));
  Serial.print("Used: "); Serial.println(humanReadableSize(LittleFS.usedBytes()));
  freeSpace = freeSpace + "\n" + "Used: " + humanReadableSize(LittleFS.usedBytes());
  Serial.print("Total: "); Serial.println(humanReadableSize(LittleFS.totalBytes()));
  freeSpace = freeSpace + "\n" + "Total: " + humanReadableSize(LittleFS.totalBytes());
  return freeSpace;
}

String listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    String filesListing = "";
    String dirName = dirName.c_str();
    File root = fs.open(dirname);
    if(!root){
        Serial.println("- failed to open directory");
        return "- failed to open directory";
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return "- failed to open directory";
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            filesListing = filesListing + "\n" + "  DIR : ";
            filesListing = filesListing + "\n" + file.name();
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            filesListing = filesListing + "\n" + "  FILE: " + file.name();
            filesListing = filesListing + "\t SIZE: " + file.size();
        }
        file = root.openNextFile();
    }

    if(levels == 0){
      return filesListing;
    }else{
      return filesListing;
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
                } else if (fileName.endsWith(".js.map")){
                  fileJsMap.concat(file.name());
                } else if (fileName.endsWith(".css")){
                  fileCss.concat(file.name());
                } else if (fileName.endsWith(".css.map")){
                  fileCssMap.concat(file.name());
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

void saveConfigDataToJSON(){
  configDoc["asAP"] = config.asAP;
  configDoc["clientSSID"] = config.clientSSID;
  configDoc["clientPasswd"] = config.clientPasswd;
  configDoc["connectionTimeOut"] = config.connectionTimeOut;
  configDoc["deviceName"] = config.deviceName;
  configDoc["apPasswd"] = config.apPasswd;
  configDoc["httpPort"] = config.http_port;
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
  configDoc["selectedMap"] = String(config.selectedMap);
}

// Saves the configuration to a file
void saveConfiguration(fs::FS &fs, const char * path) {
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
  tempClientSSID.toCharArray(config.clientSSID, 25);
  String tempClientPasswd = configDoc["clientPasswd"];
  tempClientPasswd.toCharArray(config.clientPasswd, 16);
  config.connectionTimeOut = configDoc["connectionTimeOut"].as<int>();
  String tempDeviceName = configDoc["deviceName"];
  tempDeviceName.toCharArray(config.deviceName, 16);
  String tempApPasswd = configDoc["apPasswd"];
  tempApPasswd.toCharArray(config.apPasswd, 16);
  config.http_port = configDoc["httpPort"].as<int>();
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
  config.selectedMap = configDoc["selectedMap"].as<int>();
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
    saveConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str());
  }else{
  putJSONConfigDataInMemory();
  file.close();
  }
}

void saveCalibrationDataToJSON(){
  calDataDoc["calibrateMag"] = caldata.calibrateMag;
  calDataDoc["compassOffset"] = String(caldata.compassOffset,2);
  calDataDoc["magBiasX"] = String(caldata.magBiasX);
  calDataDoc["magBiasY"] = String(caldata.magBiasY);
  calDataDoc["magBiasZ"] = String(caldata.magBiasZ);  
  calDataDoc["magScaleFacX"] = String(caldata.magScaleFacX,2);
  calDataDoc["magScaleFacY"] = String(caldata.magScaleFacY,2);
  calDataDoc["magScaleFacZ"] = String(caldata.magScaleFacZ,2);  
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
  caldata.calibrateMag = calDataDoc["calibrateMag"].as<bool>();
  caldata.compassOffset = calDataDoc["compassOffset"].as<float>();
  caldata.magBiasX = calDataDoc["magBiasX"].as<int>();
  caldata.magBiasY = calDataDoc["magBiasY"].as<int>();
  caldata.magBiasZ = calDataDoc["magBiasZ"].as<int>();  
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

// map management
String prepMapNameForMapDB(String fileName){
  String response = "";
    if(fileName.endsWith(".png")){
      uploadedPNGFile = fileName;
    }else if (fileName.endsWith(".kml")){
      uploadedKMLFile = fileName;
    if(debugSettings.debug2Serial){
      Serial.println(uploadedPNGFile + " uploaded.");
      Serial.println(uploadedKMLFile + " uploaded.");
    }
      response = "Files uploaded.";
    }else{
      Serial.println("Wrong type of file.");
      deleteFile(LittleFS, fileName.c_str());
      response = "Wrong filetype.";
    }
    return response; 
}

// Not used
void removeMapFromDB(String filenName, int id){
    Serial.println("Removing map from DB.");
}

void putJSONSelectedMapInMemory(JsonObject map) {
  selectedMap.map_id = map["id"];
  String temp = map["name"];
  temp.toCharArray(selectedMap.map_name, 32);
  String temp2 = map["area"];
  temp.toCharArray(selectedMap.map_area, 32);
  String temp3 = map["country"];
  temp.toCharArray(selectedMap.map_country, 64);
  String temp4 = map["pngFile"];
  temp.toCharArray(selectedMap.map_pngFile, 32);
  selectedMap.imageWidth = map["imageWidth"];
  selectedMap.imageHeight = map["imageHeight"];
  String temp5 = map["kmlFile"];
  temp.toCharArray(selectedMap.map_kmlFile, 32);
  selectedMap.realWorldHeight = map["realWorldHeight"];
  selectedMap.realWorldWidth = map["realWorldWidth"];
  selectedMap.scaleHeight = map["scaleHeight"];
  selectedMap.scaleWidth = map["scaleWidth"];
  selectedMap.north = map["north"];
  selectedMap.west = map["west"];
  selectedMap.south = map["south"];
  selectedMap.east = map["east"];
  selectedMap.rotation = map["rotation"];
  selectedMap.radius = map["radius"];
  }

void saveMapList(fs::FS &fs, const char * path) {
  deleteFile(LittleFS, path);
  Serial.println(path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
    return;
  }
  if (serializeJson(mapListDoc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }else{
    Serial.println(F("New file written"));
  }
  file.close();
}

void writeMapToJSON(fs::FS &fs, const char * path, int requestedMap, String name, String area, String country, String pngFile, int imageWidth, int imageHeight,String kmlFile,
                    double realWorldHeight, double realWorldWidth, float scaleHeight, float scaleWidth, double north, double west, double south, double east, float rotation, int radius){
  Serial.println("Writing map from recieved data.");
  File file = fs.open(path, FILE_READ);
  DeserializationError error = deserializeJson(mapListDoc, file);
  file.close(); 

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  for (JsonObject map : mapListDoc["maps"].as<JsonArray>()) {
    int map_id = map["id"];
    if(requestedMap == map_id){
      map["name"] = name;
      map["area"] = area;
      map["country"] = country;
      map["pngFile"] = pngFile;
      map["imageWidth"] = imageWidth;
      map["imageHeight"] = imageHeight;
      map["kmlFile"] = kmlFile;
      map["realWorldHeight"] = realWorldHeight;
      map["realWorldWidth"] = realWorldWidth;
      map["scaleHeight"] = scaleHeight;
      map["scaleWidth"] = scaleWidth;
      map["north"] = north;
      map["west"] = west;
      map["south"] = south;
      map["east"] = east;
      map["rotation"] = rotation;
      map["radius"] = radius;
    }
  }
  if(debugSettings.debug2Serial){
    serializeJson(mapListDoc, Serial);
  }
  saveMapList(LittleFS, path);
  Serial.println("");
}

void readMapFromJSON(fs::FS &fs, const char * path, int requestedMap){
  //Serial.println("Reading maps from mapData.json");
  File file = fs.open(path, FILE_READ);
  DeserializationError error = deserializeJson(mapListDoc, file);

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  for (JsonObject map : mapListDoc["maps"].as<JsonArray>()) {
    int map_id = map["id"];
    if(requestedMap == map_id){
      selectedMapJSON["id"] = map["id"];
      selectedMapJSON["name"] = map["name"];
      selectedMapJSON["area"] = map["area"];
      selectedMapJSON["country"] = map["country"];
      selectedMapJSON["pngFile"] = map["pngFile"];
      selectedMapJSON["imageWidth"] = map["imageWidth"];
      selectedMapJSON["imageHeight"] = map["imageHeight"];
      selectedMapJSON["kmlFile"] = map["kmlFile"];
      selectedMapJSON["realWorldHeight"] = map["realWorldHeight"];
      selectedMapJSON["realWorldWidth"] = map["realWorldWidth"];
      selectedMapJSON["scaleHeight"] = map["scaleHeight"];
      selectedMapJSON["scaleWidth"] = map["scaleWidth"];
      selectedMapJSON["north"] = map["north"];
      selectedMapJSON["west"] = map["west"];
      selectedMapJSON["south"] = map["south"];
      selectedMapJSON["east"] = map["east"];
      selectedMapJSON["rotation"] = map["rotation"];
      selectedMapJSON["radius"] = map["radius"];
    }
  }
}

void IRAM_ATTR readAllMapsFromJSON(fs::FS &fs, const char * path){
  Serial.println("Reading maps from mapData.json");
  File file = fs.open(path, FILE_READ);
  DeserializationError error = deserializeJson(mapListDoc, file);
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }
  for (JsonObject map : mapListDoc["maps"].as<JsonArray>()) {
    int map_id = map["id"];
    const char* map_name = map["name"];
    const char* map_area = map["area"];
    const char* map_country = map["country"];
    const char* map_pngFile = map["pngFile"];
    const char* map_kmlFile = map["kmlFile"];
  }
}

void addMaptoDB(String PNGFile, String KMLFile, JsonObject obj){
  int id = obj["id"];
  mapSelector = id;
  String name = obj["name"];
  String area = obj["area"];
  String country = obj["country"];
  PNGFile = mapsDir + "/" + PNGFile;
  int imageWidth = obj["imageWidth"];
  int imageHeight = obj["imageHeight"];
  KMLFile = mapsDir + "/" + KMLFile;
  double realWorldHeight = obj["realWorldHeight"];
  double realWorldWidth = obj["realWorldWidth"];
  float scaleHeight = obj["scaleHeight"];
  float scaleWidth = obj["scaleWidth"];
  double north = obj["north"];
  double west = obj["west"];
  double south = obj["south"];
  double east = obj["east"];
  float rotation = obj["rotation"];
  int radius = obj["radius"];
  if(debugSettings.debug2Serial){
    serializeJson(mapListDoc, Serial);
  }
  writeMapToJSON(LittleFS, (jsonDir + fileMapDataJSON).c_str(), mapSelector, name, area, country, PNGFile, imageWidth, imageHeight,  
                             KMLFile,realWorldHeight, realWorldWidth,scaleHeight, scaleWidth, north, west, south, east, rotation, radius);
};

void updateMaptoDB(String PNGFile, String KMLFile, JsonObject obj, bool pngUpdated, bool kmlUpdated){
  int id = obj["id"];
  mapSelector = id;
  String name = obj["name"];
  String area = obj["area"];
  String country = obj["country"];
  if(pngUpdated){
    PNGFile = mapsDir + "/" + PNGFile;
  }
  int imageWidth = obj["imageWidth"];
  int imageHeight = obj["imageHeight"];
  if(kmlUpdated){
    KMLFile = mapsDir + "/" + KMLFile;
  }
  double realWorldHeight = obj["realWorldHeight"];
  double realWorldWidth = obj["realWorldWidth"];
  float scaleHeight = obj["scaleHeight"];
  float scaleWidth = obj["scaleWidth"];
  double north = obj["north"];
  double west = obj["west"];
  double south = obj["south"];
  double east = obj["east"];
  float rotation = obj["rotation"];
  int radius = obj["radius"];
  if(debugSettings.debug2Serial){
    serializeJson(mapListDoc, Serial);
  }
  writeMapToJSON(LittleFS, (jsonDir + fileMapDataJSON).c_str(), mapSelector, name, area, country, PNGFile, imageWidth, imageHeight,  
                             KMLFile,realWorldHeight, realWorldWidth,scaleHeight, scaleWidth, north, west, south, east, rotation, radius);
};

// Waypoints - json doc: waypointsMapsDoc  json array nested: mapWaypoints
void readMapWaypointsFromJSON(fs::FS &fs, const char * path, int requestedMap){
  //Serial.println("Reading waypoints from waypoints.json");
  File file = fs.open(path, FILE_READ);
  DeserializationError error = deserializeJson(waypointsMapsDoc, file);
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  for (JsonObject mapWaypoints : waypointsMapsDoc["mapWaypoints"].as<JsonArray>()) {
    int map_id = mapWaypoints["mapId"];
    if(requestedMap == map_id){
      for (int i = 0; i < 30; i++) {
        String wpElement = "wp";
        wpElement.concat(i+1);
        //Serial.println(wpElement);
        wayPoints.wayPoint[i][0] = mapWaypoints[wpElement][0].as<double>();
        wayPoints.wayPoint[i][1] = mapWaypoints[wpElement][1].as<double>();
      }      
      //Serial.println("requested:"); 
      //Serial.println(map_id);
      for (int i = 0; i < 30; i++) {
        //Serial.println(wayPoints.wayPoint[i][0],8);
        //Serial.println(wayPoints.wayPoint[i][1],8);
      }
    }
  }
}

// GPS functions
static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do 
  {
    while (Serial2.available()){
      if(debugSettings.debugGPS2Serial){
        while (Serial2.available() > 0 && debugSettings.debugGPS2Serial) {    
          Serial.write(Serial2.read()); 
        }
        while (Serial.available() > 0 && debugSettings.debugGPS2Serial) {    
          Serial2.write(Serial.read()); 
        }
        gps.encode(Serial2.read());
      }else{
        gps.encode(Serial2.read());
      }
    }
  } 
  while (millis() - start < ms);
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

//  Read MPU 6050 data
void read_mpu_6050_data()
{
  /*
    Subroutine for reading the raw gyro and accelerometer data
  */

  // ----- Locals
  int     temperature;                                  // Needed when reading the MPU-6050 data ... not used

  // ----- Point to data
  Wire.beginTransmission(0x68);                         // Start communicating with the MPU-6050
  Wire.write(0x3B);                                     // Point to start of data
  Wire.endTransmission();                               // End the transmission

  // ----- Read the data
  Wire.requestFrom(0x68, 14);                           // Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                        // Wait until all the bytes are received
  Accel_x = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_x variable
  Accel_y = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_y variable
  Accel_z = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_z variable
  temperature = Wire.read() << 8 | Wire.read();         // Combine MSB,LSB temperature variable
  Gyro_x = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
  Gyro_y = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
  Gyro_z = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
}

//  Configure magnetometer
void configure_magnetometer()
{

  /*
     The MPU-9250 contains an AK8963 magnetometer and an
     MPU-6050 gyro/accelerometer within the same package.

     To access the AK8963 magnetometer chip The MPU-9250 I2C bus
     must be changed to pass-though mode. To do this we must:
      - disable the MPU-9250 slave I2C and
      - enable the MPU-9250 interface bypass mux
  */
  // ----- Disable MPU9250 I2C master interface
  Wire.beginTransmission(MPU9250_I2C_address);                      // Open session with MPU9250
  Wire.write(MPU9250_I2C_master_enable);                            // Point USER_CTRL[5] = I2C_MST_EN
  Wire.write(0x00);                                                 // Disable the I2C master interface
  Wire.endTransmission();

  // ----- Enable MPU9250 interface bypass mux
  Wire.beginTransmission(MPU9250_I2C_address);                      // Open session with MPU9250
  Wire.write(MPU9250_Interface_bypass_mux_enable);                  // Point to INT_PIN_CFG[1] = BYPASS_EN
  Wire.write(0x02);                                                 // Enable the bypass mux
  Wire.endTransmission();

  // ----- Access AK8963 fuse ROM
  /* The factory sensitivity readings for the XYZ axes are stored in a fuse ROM.
     To access this data we must change the AK9863 operating mode.
  */
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // CNTL[3:0] mode bits
  Wire.write(0b00011111);                                           // Output data=16-bits; Access fuse ROM
  Wire.endTransmission();
  delay(100);                                                       // Wait for mode change

  // ----- Get factory XYZ sensitivity adjustment values from fuse ROM
  /* There is a formula on page 53 of "MPU-9250, Register Map and Decriptions, Revision 1.4":
      Hadj = H*(((ASA-128)*0.5)/128)+1 where
      H    = measurement data output from data register
      ASA  = sensitivity adjustment value (from fuse ROM)
      Hadj = adjusted measurement data (after applying
  */

  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_fuse_ROM);                                      // Point to AK8963 fuse ROM
  Wire.endTransmission();
  Wire.requestFrom(AK8963_I2C_address, 3);                          // Request 3 bytes of data
  while (Wire.available() < 3);                                     // Wait for the data
  caldata.ASAX = (Wire.read() - 128) * 0.5 / 128 + 1;               // Adjust data
  caldata.ASAY = (Wire.read() - 128) * 0.5 / 128 + 1;
  caldata.ASAZ = (Wire.read() - 128) * 0.5 / 128 + 1;

  // ----- Power down AK8963 while the mode is changed
  /*
     This wasn't necessary for the first mode change as the chip was already powered down
  */
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // Point to mode control register
  Wire.write(0b00000000);                                           // Set mode to power down
  Wire.endTransmission();
  delay(100);                                                       // Wait for mode change

  // ----- Set output to mode 2 (16-bit, 100Hz continuous)
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // Point to mode control register
  Wire.write(0b00010110);                                           // Output=16-bits; Measurements = 100Hz continuous
  Wire.endTransmission();
  delay(100);                                                       // Wait for mode change
}
//  Calibrate magnetometer
void calibrate_magnetometer()
{
      // ----- Record magnetometer offsets
  /*
     When active this feature sends the magnetometer data
     to the Serial Monitor then halts the program.
  */
  if (caldata.calibrateMag == true)
  {
  Serial.println("Calibrating magnetometer... ");
  // ----- Locals
  int mag_x, mag_y, mag_z;
  int status_reg_2;                                               // ST2 status register

  int mag_x_min =  0;                                         // Raw data extremes
  int mag_y_min =  0;
  int mag_z_min =  0;
  int mag_x_max = 65536;
  int mag_y_max = 65536;
  int mag_z_max = 65536;

  float chord_x,  chord_y,  chord_z;                              // Used for calculating scale factors
  float chord_average;

  // ----- Record min/max XYZ compass readings
  for (int counter = 0; counter < 16000 ; counter ++)             // Run this code 16000 times
  {
    Loop_start = micros();                                        // Start loop timer

    // ----- Point to status register 1
    Wire.beginTransmission(AK8963_I2C_address);                   // Open session with AK8963
    Wire.write(AK8963_status_reg_1);                              // Point to ST1[0] status bit
    Wire.endTransmission();
    Wire.requestFrom(AK8963_I2C_address, 1);                      // Request 1 data byte
    while (Wire.available() < 1);                                 // Wait for the data
    if (Wire.read() & AK8963_data_ready_mask)                     // Check data ready bit
    {
      // ----- Read data from each axis (LSB,MSB)
      Wire.requestFrom(AK8963_I2C_address, 7);                    // Request 7 data bytes
      while (Wire.available() < 7);                               // Wait for the data
      mag_x = (Wire.read() | Wire.read() << 8) * caldata.ASAX;            // Combine LSB,MSB X-axis, apply ASA corrections
      mag_y = (Wire.read() | Wire.read() << 8) * caldata.ASAY;            // Combine LSB,MSB Y-axis, apply ASA corrections
      mag_z = (Wire.read() | Wire.read() << 8) * caldata.ASAZ;            // Combine LSB,MSB Z-axis, apply ASA corrections
      status_reg_2 = Wire.read();                                 // Read status and signal data read

      // ----- Validate data
      if (!(status_reg_2 & AK8963_overflow_mask))                 // Check HOFL flag in ST2[3]
      {
        // ----- Find max/min xyz values
        mag_x_min = min(mag_x, mag_x_min);
        mag_x_max = max(mag_x, mag_x_max);
        mag_y_min = min(mag_y, mag_y_min);
        mag_y_max = max(mag_y, mag_y_max);
        mag_z_min = min(mag_z, mag_z_min);
        mag_z_max = max(mag_z, mag_z_max);
      }
    }
    delay(4);                                                     // Time interval between magnetometer readings
  }

  // ----- Calculate hard-iron offsets
  int Mag_x_offset = (mag_x_max + mag_x_min) / 2;                     // Get average magnetic bias in counts
  int Mag_y_offset = (mag_y_max + mag_y_min) / 2;
  int Mag_z_offset = (mag_z_max + mag_z_min) / 2;

  // ----- Calculate soft-iron scale factors
  chord_x = ((float)(mag_x_max - mag_x_min)) / 2;                 // Get average max chord length in counts
  chord_y = ((float)(mag_y_max - mag_y_min)) / 2;
  chord_z = ((float)(mag_z_max - mag_z_min)) / 2;

  chord_average = (chord_x + chord_y + chord_z) / 3;              // Calculate average chord length

  float Mag_x_scale = chord_average / chord_x;                          // Calculate X scale factor
  float Mag_y_scale = chord_average / chord_y;                          // Calculate Y scale factor
  float Mag_z_scale = chord_average / chord_z;                          // Calculate Z scale factor

  caldata.magBiasX = Mag_x_offset;
  caldata.magBiasY = Mag_y_offset;
  caldata.magBiasZ = Mag_z_offset;
  caldata.magScaleFacX = Mag_x_scale;
  caldata.magScaleFacY = Mag_y_scale;
  caldata.magScaleFacZ = Mag_z_scale;

  // ----- Record magnetometer offsets
  /*
     When active this feature sends the magnetometer data
     to the Serial Monitor then halts the program.
  */
  // if (caldata.calibrateMag == true)
  // {
    // ----- Display data extremes
    Serial.print("XYZ Max/Min: ");
    Serial.print(mag_x_min); Serial.print("\t");
    Serial.print(mag_x_max); Serial.print("\t");
    Serial.print(mag_y_min); Serial.print("\t");
    Serial.print(mag_y_max); Serial.print("\t");
    Serial.print(mag_z_min); Serial.print("\t");
    Serial.println(mag_z_max);
    Serial.println("");

    // ----- Display hard-iron offsets
    Serial.print("Hard-iron: ");
    Serial.print(caldata.magBiasX); Serial.print("\t");
    Serial.print(caldata.magBiasY); Serial.print("\t");
    Serial.println(caldata.magBiasZ);
    Serial.println("");

    // ----- Display soft-iron scale factors
    Serial.print("Soft-iron: ");
    Serial.print(caldata.magScaleFacX); Serial.print("\t");
    Serial.print(caldata.magScaleFacY); Serial.print("\t");
    Serial.println(caldata.magScaleFacZ);
    Serial.println("");

    // ----- Display fuse ROM values
    Serial.print("ASA: ");
    Serial.print(caldata.ASAX); Serial.print("\t");
    Serial.print(caldata.ASAY); Serial.print("\t");
    Serial.println(caldata.ASAZ);

    // ----- Halt program
    //while (true);                                       // Wheelspin ... program halt
    saveCalibrationData(LittleFS, (jsonDir + fileCalDataJSON).c_str(), caldata);
  }
}
//  Read magnetometer
void read_magnetometer()
{
  // ----- Locals
  int mag_x, mag_y, mag_z;
  int status_reg_2;
  float ASAX = caldata.ASAX;
  float ASAY = caldata.ASAY;
  float ASAZ = caldata.ASAZ;
  float Mag_x_scale = caldata.magScaleFacX;
  float Mag_y_scale = caldata.magScaleFacY;
  float Mag_z_scale = caldata.magScaleFacZ;
  int Mag_x_offset = caldata.magBiasX;
  int Mag_y_offset = caldata.magBiasY;
  int Mag_z_offset = caldata.magBiasZ;

  // ----- Point to status register 1
  Wire.beginTransmission(AK8963_I2C_address);                   // Open session with AK8963
  Wire.write(AK8963_status_reg_1);                              // Point to ST1[0] status bit
  Wire.endTransmission();
  Wire.requestFrom(AK8963_I2C_address, 1);                      // Request 1 data byte
  while (Wire.available() < 1);                                 // Wait for the data
  if (Wire.read() & AK8963_data_ready_mask)                     // Check data ready bit
  {
    // ----- Read data from each axis (LSB,MSB)
    Wire.requestFrom(AK8963_I2C_address, 7);                    // Request 7 data bytes
    while (Wire.available() < 7);                               // Wait for the data
    mag_x = (Wire.read() | Wire.read() << 8) * ASAX;            // Combine LSB,MSB X-axis, apply ASA corrections
    mag_y = (Wire.read() | Wire.read() << 8) * ASAY;            // Combine LSB,MSB Y-axis, apply ASA corrections
    mag_z = (Wire.read() | Wire.read() << 8) * ASAZ;            // Combine LSB,MSB Z-axis, apply ASA corrections
    status_reg_2 = Wire.read();                                 // Read status and signal data read

    // ----- Validate data
    if (!(status_reg_2 & AK8963_overflow_mask))                 // Check HOFL flag in ST2[3]
    {
      Mag_x = (mag_x - Mag_x_offset) * Mag_x_scale;
      Mag_y = (mag_y - Mag_y_offset) * Mag_y_scale;
      Mag_z = (mag_z - Mag_z_offset) * Mag_z_scale;
    }
  }
}
//  Configure the gyro & accelerometer
void config_gyro()
{
  // ----- Activate the MPU-6050
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x6B);                                     //Point to power management register
  Wire.write(0x00);                                     //Use internal 20MHz clock
  Wire.endTransmission();                               //End the transmission

  // ----- Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x1C);                                     //Point to accelerometer configuration reg
  Wire.write(0x10);                                     //Select +/-8g full-scale
  Wire.endTransmission();                               //End the transmission

  // ----- Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x1B);                                     //Point to gyroscope configuration
  Wire.write(0x08);                                     //Select 500dps full-scale
  Wire.endTransmission();                               //End the transmission
}
//  Calibrate gyro
void calibrate_gyro()
{
  // ----- LED Status (ON = calibration start)
  //pinMode(LED, OUTPUT);                                 //Set LED (pin 13) as output
  //digitalWrite(LED, HIGH);                              //Turn LED on ... indicates startup

  // ----- Calibrate gyro
  for (int counter = 0; counter < 2000 ; counter ++)    //Run this code 2000 times
  {
    Loop_start = micros();
    read_mpu_6050_data();                               //Read the raw acc and gyro data from the MPU-6050
    Gyro_x_cal += Gyro_x;                               //Add the gyro x-axis offset to the gyro_x_cal variable
    Gyro_y_cal += Gyro_y;                               //Add the gyro y-axis offset to the gyro_y_cal variable
    Gyro_z_cal += Gyro_z;                               //Add the gyro z-axis offset to the gyro_z_cal variable
    while (micros() - Loop_start < Loop_time);           // Wait until "Loop_time" microseconds have elapsed
  }
  Gyro_x_cal /= 2000;                                   //Divide the gyro_x_cal variable by 2000 to get the average offset
  Gyro_y_cal /= 2000;                                   //Divide the gyro_y_cal variable by 2000 to get the average offset
  Gyro_z_cal /= 2000;                                   //Divide the gyro_z_cal variable by 2000 to get the average offset

  caldata.gyroBiasX = Gyro_x_cal;
  caldata.gyroBiasY = Gyro_y_cal;
  caldata.gyroBiasZ = Gyro_z_cal;
  // ----- Status LED
  // digitalWrite(LED, LOW);                               // Turn LED off ... calibration complete
}

float getTiltCompensatedHeading(){
  ////////////////////////////////////////////
  //        PITCH & ROLL CALCULATIONS       //
  ////////////////////////////////////////////

  /*
     --------------------
     MPU-9250 Orientation
     --------------------
     Component side up
     X-axis facing forward
  */

  // ----- read the raw accelerometer and gyro data
  read_mpu_6050_data();                                             // Read the raw acc and gyro data from the MPU-6050
  Gyro_x_cal = caldata.gyroBiasX;
  Gyro_y_cal = caldata.gyroBiasY;
  Gyro_z_cal = caldata.gyroBiasZ;
  // ----- Adjust for offsets
  Gyro_x -= Gyro_x_cal;                                             // Subtract the offset from the raw gyro_x value
  Gyro_y -= Gyro_y_cal;                                             // Subtract the offset from the raw gyro_y value
  Gyro_z -= Gyro_z_cal;                                             // Subtract the offset from the raw gyro_z value

  // ----- Calculate travelled angles
  /*
    ---------------------------
    Adjust Gyro_xyz signs for:
    ---------------------------
    Pitch (Nose - up) = +ve reading
    Roll (Right - wing down) = +ve reading
    Yaw (Clock - wise rotation)  = +ve reading
  */
  Gyro_pitch += -Gyro_y * Sensor_to_deg;                            // Integrate the raw Gyro_y readings
  Gyro_roll += Gyro_x * Sensor_to_deg;                              // Integrate the raw Gyro_x readings
  Gyro_yaw += -Gyro_z * Sensor_to_deg;                              // Integrate the raw Gyro_x readings

  // ----- Compensate pitch and roll for gyro yaw
  Gyro_pitch += Gyro_roll * sin(Gyro_z * Sensor_to_rad);            // Transfer the roll angle to the pitch angle if the Z-axis has yawed
  Gyro_roll -= Gyro_pitch * sin(Gyro_z * Sensor_to_rad);            // Transfer the pitch angle to the roll angle if the Z-axis has yawed

  // ----- Accelerometer angle calculations
  Accel_total_vector = sqrt((Accel_x * Accel_x) + (Accel_y * Accel_y) + (Accel_z * Accel_z));   // Calculate the total (3D) vector
  Accel_pitch = asin((float)Accel_x / Accel_total_vector) * RAD_TO_DEG;                         //Calculate the pitch angle
  Accel_roll = asin((float)Accel_y / Accel_total_vector) * RAD_TO_DEG;                          //Calculate the roll angle

  // ----- Zero any residual accelerometer readings
  /*
     Place the accelerometer on a level surface
     Adjust the following two values until the pitch and roll readings are zero
  */
  Accel_pitch -= -0.2f;                                             //Accelerometer calibration value for pitch
  Accel_roll -= 1.1f;                                               //Accelerometer calibration value for roll

  // ----- Correct for any gyro drift
  if (Gyro_synchronised)
  {
    // ----- Gyro & accel have been synchronised
    Gyro_pitch = Gyro_pitch * 0.9996 + Accel_pitch * 0.0004;        //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    Gyro_roll = Gyro_roll * 0.9996 + Accel_roll * 0.0004;           //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else
  {
    // ----- Synchronise gyro & accel
    Gyro_pitch = Accel_pitch;                                       //Set the gyro pitch angle equal to the accelerometer pitch angle
    Gyro_roll = Accel_roll;                                         //Set the gyro roll angle equal to the accelerometer roll angle
    Gyro_synchronised = true;                                             //Set the IMU started flag
  }

  // ----- Dampen the pitch and roll angles
  Gyro_pitch_output = Gyro_pitch_output * 0.9 + Gyro_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  Gyro_roll_output = Gyro_roll_output * 0.9 + Gyro_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

  ////////////////////////////////////////////
  //        MAGNETOMETER CALCULATIONS       //
  ////////////////////////////////////////////
  /*
     --------------------------------
     Instructions for first time use
     --------------------------------
     Calibrate the compass for Hard-iron and Soft-iron
     distortion by temporarily setting the header to read
     bool    Record_data = true;

     Turn on your Serial Monitor before uploading the code.

     Slowly tumble the compass in all directions until a
     set of readings appears in the Serial Monitor.

     Copy these values into the appropriate header locations.

     Edit the header to read
     bool    Record_data = false;

     Upload the above code changes to your Arduino.

     This step only needs to be done occasionally as the
     values are reasonably stable.
  */

  // ----- Read the magnetometer
  read_magnetometer();

  // ----- Fix the pitch, roll, & signs
  /*
     MPU-9250 gyro and AK8963 magnetometer XY axes are orientated 90 degrees to each other
     which means that Mag_pitch equates to the Gyro_roll and Mag_roll equates to the Gryro_pitch

     The MPU-9520 and AK8963 Z axes point in opposite directions
     which means that the sign for Mag_pitch must be negative to compensate.
  */
  Mag_pitch = -Gyro_roll_output * DEG_TO_RAD;
  Mag_roll = Gyro_pitch_output * DEG_TO_RAD;

  // ----- Apply the standard tilt formulas
  Mag_x_hor = Mag_x * cos(Mag_pitch) + Mag_y * sin(Mag_roll) * sin(Mag_pitch) - Mag_z * cos(Mag_roll) * sin(Mag_pitch);
  Mag_y_hor = Mag_y * cos(Mag_roll) + Mag_z * sin(Mag_roll);

  // ----- Dampen any data fluctuations
  Mag_x_dampened = Mag_x_dampened * 0.9 + Mag_x_hor * 0.1;
  Mag_y_dampened = Mag_y_dampened * 0.9 + Mag_y_hor * 0.1;

  // ----- Calculate the heading
  Heading = atan2(Mag_x_dampened, Mag_y_dampened) * RAD_TO_DEG;  // Magnetic North

  /*
     By convention, declination is positive when magnetic north
     is east of true north, and negative when it is to the west.
  */
  Declination = config.declAngleRad * RAD_TO_DEG;
  Heading += Declination;               // Geographic North
  if (Heading > 360.0) Heading -= 360.0;
  if (Heading < 0.0) Heading += 360.0;

  // ----- Allow for under/overflow
  if (Heading < 0) Heading += 360;
  if (Heading >= 360) Heading -= 360;

  // ----- Loop control
  /*
     Adjust the loop count for a yaw reading of 360 degrees
     when the MPU-9250 is rotated exactly 360 degrees.
     (Compensates for any 16 MHz Xtal oscillator error)
  */

  // while ((micros() - Loop_start_time) < 8000);
  // Loop_start_time = micros();
  return Heading;
}

float getCompassHeading(){
  float sum = 0.0;
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat magValue = myMPU9250.getMagValues();
  float temp = myMPU9250.getTemperature();
  float resultantG = myMPU9250.getResultantG(gValue);
  previous_compassheading = compassheading;
  // Calculate heading
  float heading = atan2(magValue.y, magValue.x);
  headingraw = heading;
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
  if(caldata.compassOffset >= 0){
    if(headingDegrees < caldata.compassOffset){
      headingDegrees = headingDegrees - caldata.compassOffset + 360.0;
    }else{
      headingDegrees = headingDegrees - caldata.compassOffset;
    }
  }else{
      headingDegrees = headingDegrees + (-1.0 * caldata.compassOffset);
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

// void checkGPSFix(){
//   if(GPSFix > 1){
//     int pwm_up = 0;
//     if(!GPSFixAccepted){
//       while (pwm_up < 255){
//         ledcWrite(pwmhapticright, pwm_up); 
//         ledcWrite(pwmhapticfront, pwm_up);
//         ledcWrite(pwmhapticrear, pwm_up);
//         ledcWrite(pwmhapticleft, pwm_up);
//       delay(10);
//       pwm_up++;
//       }
//         ledcWrite(pwmhapticright, 0); 
//         ledcWrite(pwmhapticfront, 0);
//         ledcWrite(pwmhapticrear, 0);
//         ledcWrite(pwmhapticleft, 0);
//     }
//   }else{
//     ledcWrite(pwmhapticleft, 0);
//     ledcWrite(pwmhapticright, 255);
//     delay(200);
//     ledcWrite(pwmhapticright, 0); 
//     ledcWrite(pwmhapticfront, 255);
//     delay(200);
//     ledcWrite(pwmhapticfront, 0);
//     ledcWrite(pwmhapticrear, 255);
//     delay(200);
//     ledcWrite(pwmhapticrear, 0);
//     ledcWrite(pwmhapticleft, 255);
//     delay(200);
//     GPSFixAccepted = 0;     
//   }
// }

void updateSensorData(){
  if (millis() > 5000 && gps.charsProcessed() < 10){
      Serial.println(F("No GPS data received: check wiring"));
  } else {
    smartDelay(50);
    String temp;
    sensorData.gpsTime = gps.time.value();
    sensorData.ownLat = gps.location.lat();
    flCurrentLat = gps.location.lat();
    sensorData.ownLon = gps.location.lng();
    flCurrentLon = gps.location.lng();
    sensorData.compassHeading = getCompassHeading();
    //sensorData.compassHeading = getTiltCompensatedHeading();
    //Serial.println(sensorData.compassHeading);
    //Serial.println(headingraw);
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
    //checkGPSFix();
  }

  if (fix.isUpdated()){
    GPSFix = atol(fix.value());         
  }
}

void getInitialReadings(){
  smartDelay(50);
  String temp;
  sensorData.ownLat = gps.location.lat();
  flCurrentLat = gps.location.lat();
  sensorData.ownLon = gps.location.lng();
  flCurrentLon = gps.location.lng();
  sensorData.compassHeading = getCompassHeading();
  //sensorData.compassHeading = getTiltCompensatedHeading();
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

void HapticFeedbackHeading(float relativeHeading,bool bEnable,bool debug,int distancePOI){
Serial.println("HapticFeedback Called");
float front;
float right;
float rear;
float left;
int mindelay = 150;
float DelayCoefficient = (config.maxDelay - mindelay) /(config.maxDistance - config.targetReached);
float DelayOffset = mindelay - (DelayCoefficient * config.targetReached);
int distancepulse;                                                                            
bool bTargetReached = 0;
int haptic_counter = 0;
int haptic_bursts = 2;

distancepulse = distancePOI * DelayCoefficient + DelayOffset;

if(distancepulse < mindelay){
  distancepulse = mindelay;
}

if(distancepulse > 1000){
  distancepulse = 1000;
}
  
if (distancePOI < config.targetReached){
  bTargetReached = true;
}else{
  bTargetReached = false;
}
    
if (bEnable){
  if((315.0 <= relativeHeading && relativeHeading <= 359.99) || (0.0 <= relativeHeading && relativeHeading <= 44.99)){
  // front haptic on
  // 0 as midpoint ok
    if(0.0 <= relativeHeading && relativeHeading <= 44){
      front = 255.0 - (relativeHeading * b);
      right = 255.0 - ((90.0 - relativeHeading) * b);
      pwm_front = (int)front;
      pwm_right = (int)right;
      pwm_rear = 0;
      pwm_left = 0;
    }else if (315.0 <= relativeHeading && relativeHeading <= 359.99){
      front = 255.0 - ((360.0 - relativeHeading) * b);
      left = 255.0 - ((relativeHeading - 270.0) * b);
      pwm_front = (int)front;
      pwm_left = (int)left;
      pwm_rear = 0;
      pwm_right = 0; 
    }
  }
  else if(45.0 <= relativeHeading && relativeHeading <= 134.99){
  // right haptic on
  // 90 as midpoint ok
      if(relativeHeading <= 89.99){
        right = 255.0 - ((90.0 - relativeHeading) * b);
        front = 255.0 - (relativeHeading * b);
        pwm_right = (int)right;
        pwm_front = (int)front;
        pwm_rear = 0;
        pwm_left = 0;
      }
      else if(relativeHeading >= 90.0){
        right = 255.0 - ((relativeHeading - 90.0) * b);
        rear = 255.0 - ((180.0 - relativeHeading) * b);
        pwm_right = (int)right;
        pwm_rear = (int)rear;
        pwm_front = 0;
        pwm_left = 0;
      }
  }
  else if(135.0 <= relativeHeading && relativeHeading <= 224.99 ){
  // rear haptic on
  // 180 as midpoint ok
      if(relativeHeading <= 179.99){
        rear = 255.0 - ((180.0 - relativeHeading) * b);
        right = 255.0 - ((relativeHeading - 90.0) * b);          
        pwm_rear = (int)rear;
        pwm_right = (int)right;
        pwm_front = 0;
        pwm_left = 0;
      }
      else if(relativeHeading >= 180.0){
        rear = 255.0 - ((relativeHeading - 180.0) * b);
        left = 255.0 - ((270.0 - relativeHeading) * b);
        pwm_rear = (int)rear;
        pwm_left = (int)left;
        pwm_front = 0;
        pwm_left = 0;
      }    
  }
  else if(225.0 <= relativeHeading && relativeHeading <= 314.99 ){
  // left haptic on
  //270 as midpoint ok
      if(relativeHeading <= 269.99){
        left = 255.0 - ((270.0 - relativeHeading) * b);
        rear = 255.0 - ((relativeHeading - 180) * b);
        pwm_left = (int)left;
        pwm_rear = (int)rear;
        pwm_front = 0;
        pwm_right = 0;
      }
      else if(relativeHeading >= 270.0){
        left = 255.0 - ((relativeHeading - 270.0) * b);
        front = 255.0 - ((360.0 - relativeHeading) * b);
        pwm_left = (int)left;
        pwm_front = (int)front;
        pwm_rear = 0;
        pwm_right = 0;
      }
    }
  }else if (debug){
      pwm_left = 255;
      pwm_front = 255;
      pwm_rear = 255;
      pwm_right = 255;
  }
  else {
      pwm_left = 0;
      pwm_front = 0;
      pwm_rear = 0;
      pwm_right = 0;
  }

  if (bTargetReached){
    while (haptic_counter < haptic_bursts){
      ledcWrite(pwmhapticright, 255); 
      ledcWrite(pwmhapticfront, 255);
      ledcWrite(pwmhapticrear, 255);
      ledcWrite(pwmhapticleft, 255);
      delay(75);
      ledcWrite(pwmhapticright, 0); 
      ledcWrite(pwmhapticfront, 0);
      ledcWrite(pwmhapticrear, 0);
      ledcWrite(pwmhapticleft, 0);
      delay(75);
      haptic_counter++;
    }
  }else{
    if( distancePOI < config.maxDistance){
      while (haptic_counter < haptic_bursts){
          ledcWrite(pwmhapticright, pwm_right); 
          ledcWrite(pwmhapticfront, pwm_front);
          ledcWrite(pwmhapticrear, pwm_rear);
          ledcWrite(pwmhapticleft, pwm_left);
        delay(distancepulse);
          ledcWrite(pwmhapticright, 0); 
          ledcWrite(pwmhapticfront, 0);
          ledcWrite(pwmhapticrear, 0);
          ledcWrite(pwmhapticleft, 0);
        delay(distancepulse);
        haptic_counter++;
      }  
    }else{
      ledcWrite(pwmhapticright, pwm_right); 
      ledcWrite(pwmhapticfront, pwm_front);
      ledcWrite(pwmhapticrear, pwm_rear);
      ledcWrite(pwmhapticleft, pwm_left);                
    }
  }
  //delay(500);   
  if (debugSettings.debug2Serial){
      Serial.print("relativeHeading: ");
      Serial.println(relativeHeading);
      Serial.print("distancePOI: ");
      Serial.println(distancePOI);
      Serial.print("Target reached: ");
      Serial.println(bTargetReached);
      Serial.print("Target reached ack: ");
      Serial.println(bTargetReachedAck);
      delay(100);                
  }
}

void switchLedUp(int led,int speed, bool color){
  if(color){
    digitalWrite(LEDCOMMON,HIGH);
  }else{
    digitalWrite(LEDCOMMON,LOW);
  }
  int pwm = 0;
  int pwm_target = 255;

  while (pwm < pwm_target){
      switch(led){
        case 0: 
          ledcWrite(pwmled0, pwm);
          break;
        case 1: 
          ledcWrite(pwmled1, pwm);
          break;
        case 2: 
          ledcWrite(pwmled2, pwm);
          break;
        case 3: 
          ledcWrite(pwmled3, pwm);
          break;
        case 4: 
          ledcWrite(pwmled4, pwm);
          break;
        case 5: 
          ledcWrite(pwmled5, pwm);
          break;
        case 6: 
          ledcWrite(pwmled6, pwm);
          break;
        case 7: 
          ledcWrite(pwmled7, pwm);
          break;             
      }
      delay(speed);
      pwm++;
     }  
  }

void switchLedDown(int led,int speed, bool color){
  if(color){
    digitalWrite(LEDCOMMON,HIGH);
  }else{
    digitalWrite(LEDCOMMON,LOW);
  }
  int pwm = 255;
  int pwm_target = 0;
  while (pwm_target < pwm){
      switch(led){
        case 0: 
          ledcWrite(pwmled0, pwm);
          break;
        case 1: 
          ledcWrite(pwmled1, pwm);
          break;
        case 2: 
          ledcWrite(pwmled2, pwm);
          break;
        case 3: 
          ledcWrite(pwmled3, pwm);
          break;
        case 4: 
          ledcWrite(pwmled4, pwm);
          break;
        case 5: 
          ledcWrite(pwmled5, pwm);
          break;
        case 6: 
          ledcWrite(pwmled6, pwm);
          break;
        case 7: 
          ledcWrite(pwmled7, pwm);
          break;             
      }
      delay(speed);
      pwm--;
     }  
  }  

// Replaces placeholder with value for webserver
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
    printFreeSpace();
  }
}

void addWebServerHeaders(){
  // Enabling CORS
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, PUT");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");
}

void webServerHandlers(){
}

// Webserver setup responses
void webServerSetup(){
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, "/index.html", String(), false, processor);
      timerRestart(timer2);
    }
  );

  webServer.on("^\\/maps\\/([a-zA-Z0-9_]+.[A-Za-z]{3})$", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String mapFile = request->pathArg(0);
     if(mapFile.endsWith(".png")){
      request->send(LittleFS, (mapsDir + "/" + mapFile).c_str(), "image/png");
     }else if(mapFile.endsWith(".kml")){
      request->send(LittleFS, (mapsDir + "/" + mapFile).c_str(), "application/vnd.google-earth.kml+xml");
     }else{
      request->send(200, "application/json", "{ \"status\": No png or kml file requested }");
     }
  });

  webServer.on("/navigation/map-list", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, "/index.html", String(), false, processor);
      timerRestart(timer2);
    }
  );

  webServer.on("/navigation/select-map", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, "/index.html", String(), false, processor);
      timerRestart(timer2);
    }
  );

  webServer.on("/navigation/waypoints", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, "/index.html", String(), false, processor);
      timerRestart(timer2);
    }
  );

  webServer.on("/system/system-info", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, "/index.html", String(), false, processor);
      timerRestart(timer2);
    }
  );

  webServer.on("/system/settingsform", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, "/index.html", String(), false, processor);
      timerRestart(timer2);
    }
  );

  webServer.on("/system/calibrationform", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, "/index.html", String(), false, processor);
      timerRestart(timer2);
    }
  );

  webServer.on("/system/debugform", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, "/index.html", String(), false, processor);
      timerRestart(timer2);
    }
  );

  webServer.on("/redirect.html", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, "/redirect.html", String(), false, processor);
    }
  );

  webServer.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      saveConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str());
      esp_restart();
    }
  );

  webServer.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
    {
      if ((request->url() == "/settings/settings_form") && (request->method() == HTTP_POST))
      {
          if (DeserializationError::Ok == deserializeJson(configDoc, (const char*)data))
          {
              JsonObject obj = configDoc.as<JsonObject>();
              putJSONConfigDataInMemory();
              saveConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str());
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
          if (DeserializationError::Ok == deserializeJson(mapListDoc, (const char*)data))
          {
              JsonObject obj = mapListDoc.as<JsonObject>();
              String pngFile = obj["pngFile"];
              String kmlFile = obj["kmlFile"]; 
              // addMaptoDB(uploadedPNGFile, uploadedKMLFile, obj);
              addMaptoDB(pngFile, kmlFile, obj);
          }
          request->send(200, "application/json", "{ \"status\": 0 }");
      } else if ((request->url() == "/navigation/update-map") && (request->method() == HTTP_POST))
      {
          if (DeserializationError::Ok == deserializeJson(mapListDoc, (const char*)data))
          {
              JsonObject obj = mapListDoc.as<JsonObject>();
              String pngFile = obj["pngFile"];
              String kmlFile = obj["kmlFile"];
              bool pngUpdated = false;
              bool kmlUpdated = false;
               if(!pngFile.startsWith("/maps/")){
                  pngUpdated = true;
               }
               if(!kmlFile.startsWith("/maps/")){
                  kmlUpdated = true;
               }
            if(debugSettings.debug2Serial){
              Serial.println(pngFile);
              Serial.println(kmlFile);
            }
              updateMaptoDB(pngFile, kmlFile, obj, pngUpdated, kmlUpdated);
          }
          request->send(200, "application/json", "{ \"status\": 0 }");          
      } else if ((request->url() == "/navigation/request-map") && (request->method() == HTTP_POST))
      {
          String output = "{ \"status\": 0 }";
          if (DeserializationError::Ok == deserializeJson(selectedMapJSON, (const char*)data))
          {
              JsonObject obj = selectedMapJSON.as<JsonObject>();
              readMapFromJSON(LittleFS, (jsonDir + fileMapDataJSON).c_str(), obj["id"]);
              //putJSONSelectedMapInMemory(obj);
              output = "";
              serializeJson(selectedMapJSON, output);
            if(debugSettings.debug2Serial){
              Serial.println(selectedMap.map_pngFile);
            }
          }
          request->send(200, "application/json", output);
      } else if ((request->url() == "/navigation/clear-map") && (request->method() == HTTP_POST))
      {
          if (DeserializationError::Ok == deserializeJson(selectedMapJSON, (const char*)data))
          {
              JsonObject obj = selectedMapJSON.as<JsonObject>();
              String pngFile = obj["pngFile"];
              String kmlFile = obj["kmlFile"];
              deleteFile(LittleFS, (pngFile).c_str());
              deleteFile(LittleFS, (kmlFile).c_str());
              addMaptoDB("NoMap.png","NoMap.kml", obj);
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
      String fileListing = listDir(LittleFS, "/", 2);
      request->send(200, "application/json", "{ \"listing\": " + fileListing + "}");
    }
  );

  webServer.on("^\\/deletePNGFile\\/([a-zA-Z0-9_]+.[A-Za-z]{3})$", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String PNGFile = request->pathArg(0);
    deleteFile(LittleFS, (mapsDir + "/" + PNGFile).c_str());
    request->send(200, "application/json", "{ \"status\": 0 }");
  });

  webServer.on("^\\/deleteKMLFile\\/([a-zA-Z0-9_]+.[A-Za-z]{3})$", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String KMLFile = request->pathArg(0);
    deleteFile(LittleFS, (mapsDir + "/" + KMLFile).c_str());
    request->send(200, "application/json", "{ \"status\": 0 }");
  });

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

  webServer.on("/getGPSPosition", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      String lat = String(sensorData.ownLat, 8);
      String lon = String(sensorData.ownLon,8);

      request->send(200, "application/json", "{ \"GPSLat\" : " + lat + ", \"GPSLon\" : " + lon +  "}" );
    }
  );

  webServer.on("/getCompassHeading", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      String compassHeading = String(sensorData.compassHeading);
      request->send(200, "application/json", "{ \"CompassHeading\" : " + compassHeading + "}" );
    }
  );  

  webServer.on("/getGPSPositionCompassHeading", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      String lat = String(sensorData.ownLat, 8);
      String lon = String(sensorData.ownLon,8);
      String compassHeading = String(sensorData.compassHeading);
      request->send(200, "application/json", "{ \"GPSLat\" : " + lat + ", \"GPSLon\" : " + lon + ", \"CompassHeading\" : " + compassHeading + "}" );
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

  webServer.on("/upload-file", HTTP_POST, [](AsyncWebServerRequest * request) {
    request->send(200);
  }, handleUpload);

  webServer.onFileUpload(handleUpload);

  webServer.onNotFound( []( AsyncWebServerRequest * request )
    {
    if (request->method() == HTTP_OPTIONS) {
      request->send(200);
      //timerRestart(timer2);
    } else {
      request->send(LittleFS, "/redirect.html",  String(), false, processor);
      //timerRestart(timer2);
      //Serial.println("redirect called");
    }
  });
  
  addWebServerHeaders();
  webServerHandlers();
  webServer.begin();
  Serial.print("HTTP Started on port: ");
  Serial.println(config.http_port);
  delay(1000);
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

  timerAlarmEnable(timer0);
  timerAlarmEnable(timer1);
  //timerAlarmEnable(timer2);

  timerStart(timer0);
  timerStart(timer1);
  //timerStart(timer2);  
}

void PWMSetup(){
//PWM setup
  // ledcSetup(pwmhapticfront, hapticfreq, resolution);
  // ledcSetup(pwmhapticright, hapticfreq, resolution);
  // ledcSetup(pwmhapticrear, hapticfreq, resolution);
  // ledcSetup(pwmhapticleft, hapticfreq, resolution);
  // ledcAttachPin(hapticfront, pwmhapticfront);
  // ledcAttachPin(hapticright, pwmhapticright);
  // ledcAttachPin(hapticrear, pwmhapticrear);
  // ledcAttachPin(hapticleft, pwmhapticleft);

  ledcSetup(pwmled0, hapticfreq, resolution);
  ledcSetup(pwmled1, hapticfreq, resolution);
  ledcSetup(pwmled2, hapticfreq, resolution);
  ledcSetup(pwmled3, hapticfreq, resolution);
  ledcSetup(pwmled4, hapticfreq, resolution);
  ledcSetup(pwmled5, hapticfreq, resolution);
  ledcSetup(pwmled6, hapticfreq, resolution);
  ledcSetup(pwmled7, hapticfreq, resolution);
  ledcAttachPin(led0, pwmled0);
  ledcAttachPin(led1, pwmled1);
  ledcAttachPin(led2, pwmled2);
  ledcAttachPin(led3, pwmled3);
  ledcAttachPin(led4, pwmled4);
  ledcAttachPin(led5, pwmled5);
  ledcAttachPin(led6, pwmled6);        
  ledcAttachPin(led7, pwmled7);  
}

void wifiSetup(){
delay(500);

if (asAP) {
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
  ipAddress = IP.toString();
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
        saveConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str());
        delay(1000);
        ESP.restart();          
      }
    }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(config.clientSSID);
  IPAddress IP = WiFi.localIP();
  ipAddress = IP.toString();
  }
hostAddress = "http://" + ipAddress;
}

void tiltCompensatedCompassSetup(){
    // ----- Serial communication
  Wire.begin();                                         //Start I2C as master
  Wire.setClock(400000);
  // ----- Provision to disable tilt stabilization
  /*
     Connect a jumper wire between A0 and GRN to disable the "tilt stabilazation"
  */


  // ----- Status LED
 // pinMode(LED, OUTPUT);                                 // Set LED (pin 13) as output
  // digitalWrite(LED, LOW);                               // Turn LED off

  // ----- Configure the magnetometer
  configure_magnetometer();

  // ----- Calibrate the magnetometer
  /*
     Calibrate only needs to be done occasionally.
     Enter the magnetometer values into the "header"
     then set "Record_data = false".
  */
  if (caldata.calibrateMag == true)
  {
    calibrate_magnetometer();
  }

  // ----- Configure the gyro & magnetometer
  config_gyro();

  calibrate_gyro();

  Debug_start_time = micros();                          // Controls data display rate to Serial Monitor
  Loop_start_time = micros();                           // Controls the Gyro refresh rate
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

void ioSetup(){
// Initialize outputs
  //pinMode(led, OUTPUT);
  pinMode(LEDCOMMON, OUTPUT);
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);
  pinMode(LED7, OUTPUT);
  // pinMode(dta_rdy_pin,INPUT);
}

void littleFSSetup(){
  // Initialize LittleFS
    delay(1000);
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
}

void haptiCapReady(){
  Serial.print("HaptiCap ready @ ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println(" MHz");
  Serial.println(getGPSTimeMinsSecs() + " " + getGPSDate());
  Serial.print("Satellites: ");
  Serial.println(sensorData.nrOfSatellites);
  startup = true;
}

void touchPadSetup(){
//Setup interrupt on Touch Pad 1 (GPIO0) and wake up
  touchAttachInterrupt(T0, callbackT0, config.touchThreshold);
  touchAttachInterrupt(T3, callbackT3, config.touchThreshold);
  esp_sleep_enable_touchpad_wakeup();
}

void testFunction(){
  readMapWaypointsFromJSON(LittleFS, (jsonDir + fileWayPointDataJSON).c_str(), 1);
}

void setup(){
  Serial.begin(SerialUSBBaud);
  Serial2.begin(GPSBaud);

  littleFSSetup();
  loadConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str(), config);
  asAP = config.asAP;
  loadDebugSettings(LittleFS, (jsonDir + fileDebugJSON).c_str(), debugSettings);  
  
  Serial.println();
  Serial.print("HaptiCap version: ");
  Serial.println(SWVERSION);
  Serial.println("By Chrysnet.com and Triznet.com");

  touchPadSetup();
  ioSetup();
  timerSetup();
  PWMSetup();
  wifiSetup();
  loadCalibrationData(LittleFS, (jsonDir + fileCalDataJSON).c_str(), caldata);
  delay(1000);
  //tiltCompensatedCompassSetup();
  magnometerSetup();
  getInitialReadings();

  if(!asAP){
    Serial.print("IP address: ");
    Serial.println(ipAddress);
    Serial.print("Host address: ");
    Serial.println(hostAddress);
    }else{
      dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
      dnsServer.start(config.dns_port, "*", WiFi.softAPIP());
      Serial.print("DNS Started on port.");
      Serial.println(config.dns_port);
      Serial.print("IP address: ");
      Serial.println(ipAddress);
      Serial.print("Host address: ");
      Serial.println(hostAddress);
    }

  webServerSetup();
  loadSensorData(LittleFS, (jsonDir + fileSensorDataJSON).c_str(), sensorData);
  readAllMapsFromJSON(LittleFS, (jsonDir + fileMapDataJSON).c_str());
  haptiCapReady();
  testFunction();
}

void loop(){
  if(debugSettings.debugGPS2Serial){
    smartDelay(50);
  }else{
 
    if(asAP){  
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
      sensorData.compassHeading = getCompassHeading();
      //sensorData.compassHeading = getTiltCompensatedHeading();
      if(interrupt1 > 10){
        interrupt1 = 2;
      }                 
    }


  for(i = 0; i < 8; i++){
    switchLedUp(i,5,false);
  }

  for(i = 0; i < 8; i++){
    switchLedDown(i,5,false);
  }

  for(i = 0; i < 8; i++){
    switchLedUp(i,5,true);
  }

  for(i = 0; i < 8; i++){
    switchLedDown(i,5,true);
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
}
