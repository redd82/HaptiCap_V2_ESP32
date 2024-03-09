#include "filehandler.h"

// External variables residing in main.cpp
extern String fileJs;
extern String fileJsMap;
extern String fileCss;
extern String fileCssMap;
extern String GPSTimeMinsSecs;
extern String GPSTimeMins;
extern String GPSDate;
extern String uploadedPNGFile;
extern String uploadedKMLFile;
extern String mapsDir;
extern int mapSelector;

extern String jsonDir;
extern String fileConfigJSON;
extern String fileCalDataJSON;
extern String fileSensorDataJSON;
extern String fileDebugSettingsJSON;
extern String fileMapDataJSON;
extern String fileMapListJSON;

extern JsonDocument configDoc;
extern JsonDocument calDataDoc;
extern JsonDocument sensorDataDoc;
extern JsonDocument debugSettingsDoc;
extern JsonDocument mapListDoc;
extern JsonDocument waypointsMapsDoc;
extern JsonDocument selectedMapJSON;

// Local variables
bool cssJsFileNamesConcat = false;

// Prototypes
extern Config config;
extern CalData caldata;
extern DebugSettings debugSettings;
extern SensorData sensorData;
extern SelectedMap selectedMap;
extern WaypointsMap wayPoints;

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
    printFreeSpace();
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