#include "webserverhandler.h"

extern AsyncWebServer webServer;
extern TinyGPSPlus gps;

extern hw_timer_t * timer2;

extern JsonDocument configDoc;
extern JsonDocument calDataDoc;
extern JsonDocument debugSettingsDoc;
extern JsonDocument mapListDoc;
extern JsonDocument sensorDataDoc;
extern JsonDocument selectedMapDoc;

extern Config config;
extern CalData caldata;
extern DebugSettings debugSettings;
extern SensorData sensorData;
extern SelectedMap selectedMap;

extern String fileJs;
extern String fileCss;
extern String fileJsMap;
extern String fileCssMap;
extern String hostAddress;
extern String mapsDir;
extern String jsonDir;

extern String GPSTimeMins;
extern String GPSDate;

extern String fileCalDataJSON;
extern String fileConfigJSON;
extern String fileDebugJSON;
extern String fileMapDataJSON;
extern String fileSensorDataJSON;

String str2HTML;
unsigned long ota_progress_millis = 0;

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
          if (DeserializationError::Ok == deserializeJson(selectedMapDoc, (const char*)data))
          {
              JsonObject obj = selectedMapDoc.as<JsonObject>();
              readMapFromJSON(LittleFS, (jsonDir + fileMapDataJSON).c_str(), obj["id"]);
              //putJSONSelectedMapInMemory(obj);
              output = "";
              serializeJson(selectedMapDoc, output);
            if(debugSettings.debug2Serial){
              Serial.println(selectedMap.map_pngFile);
            }
          }
          request->send(200, "application/json", output);
      } else if ((request->url() == "/navigation/clear-map") && (request->method() == HTTP_POST))
      {
          if (DeserializationError::Ok == deserializeJson(selectedMapDoc, (const char*)data))
          {
              JsonObject obj = selectedMapDoc.as<JsonObject>();
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