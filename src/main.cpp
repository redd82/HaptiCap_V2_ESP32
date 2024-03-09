#include <Arduino.h>

#include "ESPAsyncWebServer.h"
#include <ElegantOTA.h>
#include <DNSServer.h>
#include "FS.h"
#include <LittleFS.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "SensorFusion.h" //SF
#include <string>

#include "modules/wifihandler.h"
#include "modules/filehandler.h"
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
JsonDocument selectedMapJSON;
JsonDocument waypointsMapsDoc;
JsonArray maps = mapListDoc["maps"].to<JsonArray>();
JsonArray mapWaypoints = waypointsMapsDoc["mapWaypoints"].to<JsonArray>();

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
unsigned long ota_progress_millis = 0;

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

const int pwmledcommon = 8;
const int pwmled0 = 0;
const int pwmled1 = 1;
const int pwmled2 = 2;
const int pwmled3 = 3;
const int pwmled4 = 4;
const int pwmled5 = 5;
const int pwmled6 = 6;
const int pwmled7 = 7;
int intensityOffsetGreen = 20;


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
float b = (255.0/90.0);   // 256/90 (2.8333333F) pwm scaled to 90 degrees (quadrant)

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

// Make sensor and server objects
TinyGPSPlus gps;
TinyGPSCustom fix(gps, "GPGSA", 2);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
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

void switchLed(int led, int speed, int color, int intensity){
  int pwm = ledcRead(led);
  int pwm_target = intensity;
  int pwm_common = ledcRead(pwmledcommon);  
  if(color == 1){
    pwm_target = pwm_target - intensityOffsetGreen;
    ledcWrite(pwmledcommon, 0);
    ledcWrite(pwmled0, 0);
    ledcWrite(pwmled1, 0);
    ledcWrite(pwmled2, 0);
    ledcWrite(pwmled3, 0);
    ledcWrite(pwmled4, 0);
    ledcWrite(pwmled5, 0);
    ledcWrite(pwmled6, 0);
    ledcWrite(pwmled7, 0);
  }else if (color == 2){
    pwm_target = ~intensity;
    pwm_target = pwm_target + 255;
    Serial.println(pwm_target);
    ledcWrite(pwmledcommon, 255);
    ledcWrite(pwmled0, 255);
    ledcWrite(pwmled1, 255);
    ledcWrite(pwmled2, 255);
    ledcWrite(pwmled3, 255);
    ledcWrite(pwmled4, 255);
    ledcWrite(pwmled5, 255);
    ledcWrite(pwmled6, 255);
    ledcWrite(pwmled7, 255);
  }else{

  }

  while (pwm != pwm_target){
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
    if(color == 1){
      if(pwm_target > pwm){
        pwm++;
      }else{
        pwm--;
      }
    }else if(color == 2){
      if(pwm > pwm_target){
        pwm--;
      }else{
        pwm++;
      }       
    }else{

    }
    delay(speed);
  }  
}

void ledTesting(){
  switchLed(0,5,1,254);           // switch led0 on with 5 ms delay between steps, green, full intensity 
  switchLed(0,5,1,0);

  switchLed(6,5,2,254);           // switch led6 on with 5 ms delay between steps, green, full intensity 
  delay(2000);
  switchLed(6,5,2,0);

  switchLed(3,2,1,254);           // switch led3 on with 2 ms delay between steps, green, full intensity 
//  Serial.println("1");
  // for(i = 0; i < 8; i++){
  //   switchLedUp(i,5,1,255);
  // }
  // Serial.println("2");
  // for(i = 0; i < 8; i++){
  //   switchLedDown(i,5,1,0);
  // }
  // Serial.println("3");
  // for(i = 0; i < 8; i++){
  //   switchLedUp(i,5,2,255);
  // }
  // Serial.println("4");
  // for(i = 0; i < 8; i++){
  //   switchLedDown(i,5,2,0);
  // } 
   // switchLed(3,2,1,0);   
};
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

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
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
      //String fileListing = listDir(LittleFS, "/", 2);
      String fileListing = listDir(LittleFS, mapsDir.c_str(), 0);
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
  
  ElegantOTA.onStart([]() {
    Serial.println("OTA update process started.");
    // Add your initialization tasks here.
  });

  ElegantOTA.onProgress([](size_t current, size_t final) {
    Serial.printf("Progress: %u%%\n", (current * 100) / final);
  });

  ElegantOTA.onEnd([](bool success) {
    if (success) {
      Serial.println("OTA update completed successfully.");
      // Add success handling here.
    } else {
      Serial.println("OTA update failed.");
      // Add failure handling here.
    }
  });

  addWebServerHeaders();
  webServerHandlers();

  webServer.begin();
  ElegantOTA.begin(&webServer);    // Start ElegantOTA
  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);
  Serial.print("HTTP Started on port: ");
  Serial.println(config.http_port);
  delay(1000);
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

  ledcSetup(pwmledcommon, hapticfreq, resolution);
  ledcSetup(pwmled0, hapticfreq, resolution);
  ledcSetup(pwmled1, hapticfreq, resolution);
  ledcSetup(pwmled2, hapticfreq, resolution);
  ledcSetup(pwmled3, hapticfreq, resolution);
  ledcSetup(pwmled4, hapticfreq, resolution);
  ledcSetup(pwmled5, hapticfreq, resolution);
  ledcSetup(pwmled6, hapticfreq, resolution);
  ledcSetup(pwmled7, hapticfreq, resolution);
  ledcAttachPin(ledcommon, pwmledcommon);
  ledcAttachPin(led0, pwmled0);
  ledcAttachPin(led1, pwmled1);
  ledcAttachPin(led2, pwmled2);
  ledcAttachPin(led3, pwmled3);
  ledcAttachPin(led4, pwmled4);
  ledcAttachPin(led5, pwmled5);
  ledcAttachPin(led6, pwmled6);        
  ledcAttachPin(led7, pwmled7);  
}

String get_wifi_status(int status){
    switch(status){
        case WL_IDLE_STATUS:
        return "WL_IDLE_STATUS";
        case WL_SCAN_COMPLETED:
        return "WL_SCAN_COMPLETED";
        case WL_NO_SSID_AVAIL:
        return "WL_NO_SSID_AVAIL";
        case WL_CONNECT_FAILED:
        return "WL_CONNECT_FAILED";
        case WL_CONNECTION_LOST:
        return "WL_CONNECTION_LOST";
        case WL_CONNECTED:
        return "WL_CONNECTED";
        case WL_DISCONNECTED:
        return "WL_DISCONNECTED";
    }
    return "UNKNOWN_STATUS";
}

// void wifiSetup(){
//   if (config.asAP) {
//     const char *ssid = "testAP";
//     const char *password = "yourPassword";
//     WiFi.mode(WIFI_MODE_APSTA);
//     Serial.println("Setting up WiFi in AP Mode! ");
//     Serial.println(config.deviceName);
//     Serial.println(config.apPasswd);
//     WiFi.softAP(config.deviceName, config.apPasswd);

//   if (!WiFi.softAP(ssid, password)) {
//     log_e("Soft AP creation failed.");
//     while(1);
//   }    
//     // if (!WiFi.softAP(config.deviceName, config.apPasswd)) {
//     //   log_e("Soft AP creation failed.");
//     //   while(1);
//     // }
//     delay(100);
//     WiFi.setAutoReconnect(false);
//     WiFi.softAP(config.deviceName,config.apPasswd, 13, 0, 2);
//     WiFi.setTxPower(WIFI_POWER_15dBm);
//     int txPower = WiFi.getTxPower();
//     Serial.print("TX Power: ");
//     Serial.println(txPower);
//     delay(100);
//     IPAddress ip( 192, 168, 1, 1 );
//     IPAddress gateway( 192, 168, 1, 1 );
//     IPAddress subnet( 255, 255, 255, 0 );
//     delay(2000);
//     WiFi.softAPConfig( ip, gateway, subnet );  
//     IPAddress IP = WiFi.softAPIP();
//     Serial.print("AP IP address: ");
//     Serial.println(IP);
//     ipAddress = IP.toString();
//   }else{
//     Serial.print("Setting HapiCap as client to network ");
//     Serial.println(config.clientSSID);
//     //WiFi.mode(WIFI_STA);
//     WiFi.begin(config.clientSSID, config.clientPasswd);
//     intCounterWifi = 0;
      
//     while (WiFi.status() != WL_CONNECTED){
//       delay(500);
//       //Serial.print(".");
//       intCounterWifi++;
//         if (intCounterWifi > 120){
//           Serial.println("");
//           config.asAP = 1;
//           saveConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str());
//           delay(1000);
//           ESP.restart();          
//         }
//       }
//     Serial.println("");
//     Serial.print("Connected to ");
//     Serial.println(config.clientSSID);
//     IPAddress IP = WiFi.localIP();
//     ipAddress = IP.toString();
//   }
// hostAddress = "http://" + ipAddress;
// }

void magnometerSetup(){
  // start communication with IMU 
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while (1);
  }

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
  loadCalibrationData(LittleFS, (jsonDir + fileCalDataJSON).c_str(), caldata);
  delay(1000);
  //tiltCompensatedCompassSetup();
  wireScan();
  //magnometerSetup();
  getInitialReadings();
  delay(10000);
  if(!asAP){
    Serial.print("IP address: ");
    Serial.println(ipAddress);
    Serial.print("Host address: ");
    Serial.println(hostAddress);
    }else{
      //dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
      //dnsServer.start(config.dns_port, "*", WiFi.softAPIP());
      //Serial.print("DNS Started on port.");
      //Serial.println(config.dns_port);
      Serial.print("IP address: ");
      Serial.println(ipAddress);
      Serial.print("Host address: ");
      Serial.println(hostAddress);
    }

  webServerSetup();
  loadSensorData(LittleFS, (jsonDir + fileSensorDataJSON).c_str(), sensorData);
  readAllMapsFromJSON(LittleFS, (jsonDir + fileMapDataJSON).c_str());
  getInitialReadings();
  ElegantOTA.setAutoReboot(false);
  ElegantOTA.setAuth(config.deviceName, config.apPasswd);
  Serial.println("OTA Enabled!");
  haptiCapReady();
  //testFunction();
}

void loop(){
  if(debugSettings.debugGPS2Serial){
    smartDelay(50);
  }else{
 
    // if(asAP){  
    //   dnsServer.processNextRequest();
    // }

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
      //sensorData.compassHeading = getCompassHeading();
      //sensorData.compassHeading = getTiltCompensatedHeading();
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
