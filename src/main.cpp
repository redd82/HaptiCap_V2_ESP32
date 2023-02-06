#include <Arduino.h>
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include "TinyGPS++.h"
#include <ESP8266FtpServer.h>
#include <ArduinoJson.h>
#include <MPU9250_WE.h>
#include <Wire.h>
#define MPU9250_ADDR 0x68

#define MAX_SRV_CLIENTS 2
#define INPUT_SIZE 20
#define Threshold 10                                        /* Greater the value, the higher the sensitivity */

// Set declination angle on your location and fix heading
// Formula: (deg + (minutes / 60.0)) / (180 / M_PI); (4.0 + (26.0 / 60.0)) / (180 / PI);
//float declinationAngle = (declAngleDeg + (declAngleMin / 60.0)) / (180.0 / PI);

struct Config {
  uint32_t http_port = 80;
  bool As_AP = 0;
  char Client_SSID[16] = "TrizNet_AP2";
  char Client_Passwd[16] = "T0sh7b49";
  char DeviceName[24] = "HaptiCap";                       //  = "HCR-99_HaptiCap"
  char AP_Passwd[24] = "prutser00";
  unsigned long GPS_poll_sec = 1;               // Timer0 tick set to 1 s (10 * 1000 = 10000 ms) for gps 
  unsigned long Comp_poll_ms = 100;            // Timer1 tick set to 1 ms (250 * 1 = 250 ms)  for compass 
  float Comp_offset = 123.0;
  float HOME_LAT = 12.234567;
  float HOME_LON = 56.789012;
  float WAYPOINT_LAT = 2.234567;
  float WAYPOINT_LON = 6.789012;
  int intTargetReached = 10;
  int maxdistance = 500;
  int maxdelay = 1000;
  double DeclAngleRad = 0.024085543678;
  unsigned long Sleep_mins = 5;                // Timer2 tick set to 1 min (1 * 60000 = 250 ms)  sleep
  int Touch_Threshold = 50;
  int TimezoneOffset = 1;                      // +1 hour
  bool TouchEnabled = 1;
  bool DebugHaptic = 0;
  bool Debug2Telnet = 0;
  bool Debug2Serial = 0;
  bool DebugData2Serial = 0;
  bool FTPEnabled = 0;
};


// Replace with your network credentials
const char* ssid = "TrizNet_AP2";
const char* password = "T0sh7b49";

// Set LED GPIO
const int ledPin = 2;
// Stores LED state
String ledState;

// Create AsyncWebServer object on port 80
AsyncWebServer webServer(80);
FtpServer ftpSrv;
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

// Replaces placeholder with LED state value
String processor(const String& var){
  Serial.println(var);
  if(var == "STATE"){
    if(digitalRead(ledPin)){
      ledState = "ON";
    }
    else{
      ledState = "OFF";
    }
    Serial.print(ledState);
    return ledState;
  }
  return String();
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

void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

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

  // Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  } else {
        
     listDir(SPIFFS, "/", 0);
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  // Route for root / web page
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/areamap.html", String(), false, processor);
  });
  
  // Route to load style.css file
  webServer.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Route to set GPIO to HIGH
  webServer.on("/on", HTTP_GET, [](AsyncWebServerRequest *request){
    digitalWrite(ledPin, HIGH);    
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  
  // Route to set GPIO to LOW
  webServer.on("/off", HTTP_GET, [](AsyncWebServerRequest *request){
    digitalWrite(ledPin, LOW);    
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  // Start server
  webServer.begin();
  if (SPIFFS.begin(true)) {
    //ftpSrv.begin("esp8266","esp8266"); 
  }
}
 
void loop(){
   //ftpSrv.handleFTP();
}