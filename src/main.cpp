#include <Arduino.h>

#include "FS.h"
#include <LittleFS.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_Sensor.h>
#include "SensorFusion.h" //SF
#include <string>

#include "modules/pwm.h"
#include "modules/filehandler.h"
#include "modules/webserverhandler.h"
#include "modules/hapticfeedback.h"
#include "modules/wifihandler.h"
#include "modules/compass.h"
#include "modules/gps.h"

#define FORMAT_LITTLEFS_IF_FAILED true
#define SWVERSION 2.01
#define MPU9250_ADDR 0x68
#define MAX_SRV_CLIENTS 2
#define INPUT_SIZE 20
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

#define MPU9250_IMU_ADDRESS 0x68
#define MPU9250_MAG_ADDRESS 0x0C
#define GYRO_FULL_SCALE_250_DPS  0x00
#define GYRO_FULL_SCALE_500_DPS  0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18
#define ACC_FULL_SCALE_2G  0x00
#define ACC_FULL_SCALE_4G  0x08
#define ACC_FULL_SCALE_8G  0x10
#define ACC_FULL_SCALE_16G 0x18
#define TEMPERATURE_OFFSET 21 // As defined in documentation
#define INTERVAL_MS_PRINT 1000
#define G 9.80665

// refactoring main.cpp 

JsonDocument configDoc;
JsonDocument calDataDoc;
JsonDocument debugSettingsDoc;
JsonDocument sensorDataDoc;
JsonDocument mapListDoc;
JsonDocument selectedMapDoc;
JsonDocument waypointsMapsDoc;
JsonArray maps = mapListDoc["maps"].to<JsonArray>();
JsonArray mapWaypoints = waypointsMapsDoc["mapWaypoints"].to<JsonArray>();

// _Structs_
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

double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
// const int dta_rdy_pin = 19;
const int led = 23;
//const int buttonPin = 4;
const int TouchPinT0 = 4;
const int TouchPinT3 = 15;
int buttonState = 0;
touch_pad_t touchPin;
int touchValueT0;
int touchValueT3;

bool bPrintHeader = false;
int count = 0;
int i = 0;

RTC_DATA_ATTR double flCurrentLat;
RTC_DATA_ATTR double flCurrentLon;
RTC_DATA_ATTR bool bTargetReachedAck = 0;
RTC_DATA_ATTR bool bHomeReachedAck = 0;

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
bool magnetometerCalibrated = false;
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
bool compassCalibrated = false;
bool asAP = true;
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

// Make objects
TinyGPSPlus gps;
TinyGPSCustom fix(gps, "GPGSA", 2);
Adafruit_LIS3MDL lis3mdl;
Adafruit_LSM6DS3TRC lsm6ds3strc;                     //LSM6DS3TRC
SF fusion;
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

void wireScan(){
  byte error, address;
  int nDevices = 0;
  Wire.begin();
  delay(5000);
  Serial.println("Scanning for I2C devices ...");
  for(address = 0x01; address < 0x7f; address++){
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0){
      Serial.printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    } else if(error != 2){
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0){
    Serial.println("No I2C devices found");
  }
}

void updateSensorData(){
  if (millis() > 5000 && gps.charsProcessed() < 10){
      Serial.println(F("No GPS data received: check wiring"));
  } else {
    smartDelay(50);
    String temp;
    getGPSData();
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
  sensorData.compassHeading = 0;
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

void timerSetup(){
// timer0 setup
  timer0 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer0, &onTimer0, true);
  timerAlarmWrite(timer0, (config.gpsPollSec * 1000000), true);         // 1 s
// timer1 setup
  timer1 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, (config.compPollMs * 1000), true);            // 1 ms
// timer2 setup
  timer2 = timerBegin(2, 80, true);
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, (config.sleepMins * 60000000), true);         // 1 min

  timerAlarmEnable(timer0);
  timerAlarmEnable(timer1);
  //timerAlarmEnable(timer2);

  timerStart(timer0);
  timerStart(timer1);
  //timerStart(timer2);  
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
  Serial.print("IP address: ");
  Serial.println(ipAddress);
  Serial.print("Host address: ");
  Serial.println(hostAddress);
  Serial.print("HaptiCap ready @ ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println(" MHz");
  Serial.println(getGPSTimeMinsSecs() + " " + getGPSDate());
  Serial.print("Satellites: ");
  Serial.println(sensorData.nrOfSatellites);
  startup = true;
  delay(4000);
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
  Serial.println();
  Serial.print("HaptiCap version: ");
  Serial.println(SWVERSION);
  Serial.println("By Chrysnet.com and Triznet.com");

  littleFSSetup();
  loadConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str(), config);
  loadDebugSettings(LittleFS, (jsonDir + fileDebugJSON).c_str(), debugSettings);
  wifiSetup();
  touchPadSetup();
  ioSetup();
  timerSetup();
  PWMSetup();
  wireScan();
  compassSetup();
  loadCalibrationData(LittleFS, (jsonDir + fileCalDataJSON).c_str(), caldata);
  delay(1000);
  getInitialReadings();
  loadSensorData(LittleFS, (jsonDir + fileSensorDataJSON).c_str(), sensorData);
  readAllMapsFromJSON(LittleFS, (jsonDir + fileMapDataJSON).c_str());
  ElegantOTA.setAutoReboot(false);
  ElegantOTA.setAuth(config.deviceName, config.apPasswd);
  Serial.println("OTA Enabled!");
  webServerSetup();
  haptiCapReady();
}

void loop(){
  if(debugSettings.debugGPS2Serial){
    smartDelay(50);
  }else{

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
      //sensorData.compassHeading = readCompass();
      //readCompass();
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

    //ledTesting();
  }
}
