#include "webserverhandler.h"
#include "compass.h"
#include "takhandler.h"

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
extern bool runCompassCalibrationRequested;

String str2HTML;
unsigned long ota_progress_millis = 0;

static String normalizeUploadedFileName(String fileName) {
  fileName.replace("\\", "/");
  int slashIndex = fileName.lastIndexOf('/');
  if (slashIndex >= 0) {
    fileName = fileName.substring(slashIndex + 1);
  }
  return fileName;
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
  String safeFileName = normalizeUploadedFileName(filenameLocal);
  if (!index) {
    String fileForDeletion = mapsDir + "/" + safeFileName;
    LittleFS.rename(fileForDeletion.c_str(), "/maps/temp.png");
    request->_tempFile = LittleFS.open(mapsDir + "/" + safeFileName, "w");
  }
  if (len) {
    request->_tempFile.write(data, len);
  }
  if (final) {
    request->_tempFile.close();
    response = prepMapNameForMapDB(safeFileName);
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

  webServer.on("^\\/maps\\/(.+)$", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String mapFile = request->pathArg(0);
    if (mapFile.startsWith("/")) {
      mapFile.remove(0, 1);
    }

    String filePath = mapsDir + "/" + mapFile;
    if(!LittleFS.exists(filePath.c_str())){
      request->send(404, "application/json", "{ \"error\": \"Map file not found\" }");
      return;
    }

    String mapFileLower = mapFile;
    mapFileLower.toLowerCase();
    if(mapFileLower.endsWith(".png")){
      request->send(LittleFS, filePath.c_str(), "image/png");
    }else if(mapFileLower.endsWith(".kml")){
      request->send(LittleFS, filePath.c_str(), "application/vnd.google-earth.kml+xml");
    }else{
      request->send(415, "application/json", "{ \"error\": \"Unsupported map file type\" }");
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

  webServer.on("/tak_enroll.html", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(LittleFS, "/tak_enroll.html", "text/html");
    }
  );

  webServer.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      saveConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str());
      esp_restart();
    }
  );

  auto handleJsonBody = [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      if (index == 0) {
        request->_tempObject = new String();
        if (request->_tempObject == nullptr) {
          request->send(500, "application/json", "{ \"error\": \"OOM\" }");
          return;
        }
        reinterpret_cast<String*>(request->_tempObject)->reserve(total);
      }

      String *requestBody = reinterpret_cast<String*>(request->_tempObject);
      if (requestBody != nullptr && data != nullptr && len > 0) {
        requestBody->concat(reinterpret_cast<const char*>(data), len);
      }

      if ((index + len) < total) {
        return;
      }

      String body = "";
      if (requestBody != nullptr) {
        body = *requestBody;
        delete requestBody;
        request->_tempObject = nullptr;
      }

      if ((request->url() == "/settings/settings_form") && (request->method() == HTTP_POST))
      {
          if (DeserializationError::Ok == deserializeJson(configDoc, body))
          {
              JsonObject obj = configDoc.as<JsonObject>();
              putJSONConfigDataInMemory();
              saveConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str());
          }
          request->send(200, "application/json", "{ \"status\": 0 }");
      } else if ((request->url() == "/settings/calibration_form") && (request->method() == HTTP_POST))
      {
          if (DeserializationError::Ok == deserializeJson(calDataDoc, body))
          {
              JsonObject obj = calDataDoc.as<JsonObject>();
              putJSONCalibrationDataInMemory();

              bool forceCompassCalibration = false;
              if (obj["compassCalibrationMode"].is<bool>()) {
                forceCompassCalibration = obj["compassCalibrationMode"].as<bool>();
              } else if (obj["compassCalibrationMode"].is<const char*>()) {
                String mode = obj["compassCalibrationMode"].as<String>();
                mode.toLowerCase();
                forceCompassCalibration = (mode == "1" || mode == "true" || mode == "calibrate" || mode == "calibratecompass");
              }

              if (forceCompassCalibration) {
                caldata.compassCalibrated = false;
                Serial.println(F("Calibration API: forced compass recalibration requested"));
              }

              if (!caldata.compassCalibrated) {
                runCompassCalibrationRequested = true;
                Serial.println(F("Calibration API: queued runtime compass calibration"));
              } else {
                Serial.println(F("Calibration API: skipped runtime calibration (already calibrated)"));
              }

              saveCalibrationData(LittleFS, (jsonDir + fileCalDataJSON).c_str(), caldata);
          }
          request->send(200, "application/json", "{ \"status\": 0 }");
      } else if ((request->url() == "/settings/debug_form") && (request->method() == HTTP_POST))
      {
          if (DeserializationError::Ok == deserializeJson(debugSettingsDoc, body))
          {
              JsonObject obj = debugSettingsDoc.as<JsonObject>();
              putJSONDebugSettingsInMemory();
              saveDebugSettings(LittleFS, (jsonDir + fileDebugJSON).c_str(), debugSettings);
          }
          request->send(200, "application/json", "{ \"status\": 0 }");
      } else if ((request->url() == "/navigation/register-map") && (request->method() == HTTP_POST))
      {
          if (DeserializationError::Ok == deserializeJson(mapListDoc, body))
          {
              JsonObject obj = mapListDoc.as<JsonObject>();
              String pngFile = obj["pngFile"];
              String kmlFile = obj["kmlFile"]; 
              addMaptoDB(pngFile, kmlFile, obj);
          }
          request->send(200, "application/json", "{ \"status\": 0 }");
      } else if ((request->url() == "/navigation/update-map") && (request->method() == HTTP_POST))
      {
          if (DeserializationError::Ok == deserializeJson(mapListDoc, body))
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
          JsonDocument requestMapDoc;
          DeserializationError parseError = deserializeJson(requestMapDoc, body);
          if (parseError) {
            request->send(400, "application/json", "{ \"error\": \"Invalid request-map payload\", \"message\": \"Invalid request-map payload\" }");
            return;
          }

          int requestedMapId = requestMapDoc["id"] | -1;
          if (requestedMapId < 0) {
            request->send(400, "application/json", "{ \"error\": \"Missing map id\", \"message\": \"Missing map id\" }");
            return;
          }

          bool mapFound = readMapFromJSON(LittleFS, (jsonDir + fileMapDataJSON).c_str(), requestedMapId);
          if (!mapFound || selectedMapDoc["pngFile"].isNull()) {
            request->send(404, "application/json", "{ \"error\": \"Requested map not found\", \"message\": \"Requested map not found\" }");
            return;
          }

          String output = "";
          serializeJson(selectedMapDoc, output);
          if(debugSettings.debug2Serial){
            Serial.println(output);
          }
          request->send(200, "application/json", output);
      } else if ((request->url() == "/navigation/clear-map") && (request->method() == HTTP_POST))
      {
          if (DeserializationError::Ok == deserializeJson(selectedMapDoc, body))
          {
              JsonObject obj = selectedMapDoc.as<JsonObject>();
              String pngFile = obj["pngFile"];
              String kmlFile = obj["kmlFile"];
              deleteFile(LittleFS, (pngFile).c_str());
              deleteFile(LittleFS, (kmlFile).c_str());
              addMaptoDB("NoMap.png","NoMap.kml", obj);
          }
          request->send(200, "application/json", "{ \"status\": 0 }");
          } else if ((request->url() == "/tak/config") && (request->method() == HTTP_POST))
          {
            JsonDocument takPatchDoc;
            if (DeserializationError::Ok == deserializeJson(takPatchDoc, body))
            {
              mergeTAKConfigPatch(takPatchDoc);
              saveConfiguration(LittleFS, (jsonDir + fileConfigJSON).c_str());
              request->send(200, "application/json", "{ \"status\": 0 }");
            } else {
              request->send(400, "application/json", "{ \"error\": \"Invalid TAK config payload\" }");
            }
      }
  };

  // Register JSON POST endpoints with explicit body callbacks so requests are handled.
  webServer.on("/settings/settings_form", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, handleJsonBody);
  webServer.on("/settings/calibration_form", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, handleJsonBody);
  webServer.on("/settings/debug_form", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, handleJsonBody);
  webServer.on("/navigation/register-map", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, handleJsonBody);
  webServer.on("/navigation/update-map", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, handleJsonBody);
  webServer.on("/navigation/request-map", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, handleJsonBody);
  webServer.on("/navigation/clear-map", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, handleJsonBody);
  webServer.on("/tak/config", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, handleJsonBody);

  webServer.on("/tak/connect", HTTP_POST, [](AsyncWebServerRequest *request)
    {
      takConnectRequested = true;
      request->send(200, "application/json", "{ \"status\": 0 }");
    }
  );

  webServer.on("/tak/disconnect", HTTP_POST, [](AsyncWebServerRequest *request)
    {
      takDisconnectRequested = true;
      request->send(200, "application/json", "{ \"status\": 0 }");
    }
  );

  webServer.on("/tak/config", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      JsonDocument takConfigDoc;
      takConfigDoc["takEnabled"] = config.takEnabled;
      takConfigDoc["takSSL"] = config.takSSL;
      takConfigDoc["takVerifyCert"] = config.takVerifyCert;
      takConfigDoc["takPersistent"] = config.takPersistent;
      takConfigDoc["takUDPEnabled"] = config.takUDPEnabled;
      takConfigDoc["takServer"] = config.takServer;
      takConfigDoc["takPort"] = config.takPort;
      takConfigDoc["takUDPPort"] = config.takUDPPort;
      takConfigDoc["takCallsign"] = config.takCallsign;
      takConfigDoc["takUID"] = config.takUID;
      takConfigDoc["takCotType"] = config.takCotType;
      takConfigDoc["takIntervalSec"] = config.takIntervalSec;
      takConfigDoc["takCACertPath"] = config.takCACertPath;
      takConfigDoc["takEnrollPort"] = config.takEnrollPort;
      takConfigDoc["takEnrollHost"] = config.takEnrollHost;
      takConfigDoc["takEnrollUsername"] = config.takEnrollUsername;
      takConfigDoc["takEnrollToken"] = config.takEnrollToken;
      takConfigDoc["takAutoTokenFetch"] = config.takAutoTokenFetch;
      takConfigDoc["takTokenApiPath"] = config.takTokenApiPath;
      takConfigDoc["takTokenApiUsername"] = config.takTokenApiUsername;
      takConfigDoc["takTokenApiPassword"] = config.takTokenApiPassword;
      takConfigDoc["takUseClientCert"] = config.takUseClientCert;
      takConfigDoc["takEnrollPath"] = config.takEnrollPath;
      takConfigDoc["takClientCertPath"] = config.takClientCertPath;
      takConfigDoc["takClientKeyPath"] = config.takClientKeyPath;
      takConfigDoc["takClientP12Path"] = config.takClientP12Path;

      String payload;
      serializeJson(takConfigDoc, payload);
      request->send(200, "application/json", payload);
    }
  );

  webServer.on("/tak/status", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      JsonDocument statusDoc;
      statusDoc["connected"] = isTAKConnected();
      statusDoc["lastSendMs"] = getLastTAKSendMs();
      statusDoc["uid"] = config.takUID;
      statusDoc["enabled"] = config.takEnabled;

      String payload;
      serializeJson(statusDoc, payload);
      request->send(200, "application/json", payload);
    }
  );

  webServer.on("/tak/enrollment-qr-data", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      String uri = getTAKEnrollmentURI();
      JsonDocument qrDoc;
      qrDoc["host"] = (strlen(config.takEnrollHost) > 0) ? config.takEnrollHost : config.takServer;
      qrDoc["username"] = config.takEnrollUsername;
      qrDoc["token"] = config.takEnrollToken;
      qrDoc["uri"] = uri;

      String payload;
      serializeJson(qrDoc, payload);
      request->send(200, "application/json", payload);
    }
  );

  webServer.on("/tak/enrollment-qr-token/refresh", HTTP_POST, [](AsyncWebServerRequest *request)
    {
      String message;
      bool ok = refreshTAKEnrollmentToken(message);
      JsonDocument responseDoc;
      responseDoc["status"] = ok ? 0 : 1;
      responseDoc["message"] = message;
      responseDoc["username"] = config.takEnrollUsername;
      responseDoc["token"] = config.takEnrollToken;
      responseDoc["uri"] = getTAKEnrollmentURI();

      String payload;
      serializeJson(responseDoc, payload);
      request->send(ok ? 200 : 500, "application/json", payload);
    }
  );

  webServer.on("/tak/enroll", HTTP_POST, [](AsyncWebServerRequest *request)
    {
      String message;
      bool ok = enrollTAKClientCertificate(false, message);
      JsonDocument responseDoc;
      responseDoc["status"] = ok ? 0 : 1;
      responseDoc["message"] = message;

      String payload;
      serializeJson(responseDoc, payload);
      request->send(ok ? 200 : 500, "application/json", payload);
    }
  );

  webServer.on("/tak/reenroll", HTTP_POST, [](AsyncWebServerRequest *request)
    {
      String message;
      bool ok = enrollTAKClientCertificate(true, message);
      JsonDocument responseDoc;
      responseDoc["status"] = ok ? 0 : 1;
      responseDoc["message"] = message;

      String payload;
      serializeJson(responseDoc, payload);
      request->send(ok ? 200 : 500, "application/json", payload);
    }
  );

  webServer.on("/tak/enroll-status", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      JsonDocument statusDoc;
      getTAKEnrollmentStatus(statusDoc);
      String payload;
      serializeJson(statusDoc, payload);
      request->send(200, "application/json", payload);
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

  webServer.on("/file-upload", HTTP_POST, [](AsyncWebServerRequest * request) {
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