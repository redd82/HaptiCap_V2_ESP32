#include "WiFiHandler.h"

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

void wifiSetup(){
  if (config.asAP) {
    const char *ssid = "testAP";
    const char *password = "yourPassword";
    WiFi.mode(WIFI_MODE_APSTA);
    Serial.println("Setting up WiFi in AP Mode! ");
    Serial.println(config.deviceName);
    Serial.println(config.apPasswd);
    WiFi.softAP(config.deviceName, config.apPasswd);

  if (!WiFi.softAP(ssid, password)) {
    log_e("Soft AP creation failed.");
    while(1);
  }    
    delay(100);
    WiFi.setAutoReconnect(false);
    WiFi.softAP(config.deviceName,config.apPasswd, 13, 0, 2);
    WiFi.setTxPower(WIFI_POWER_15dBm);
    int txPower = WiFi.getTxPower();
    Serial.print("TX Power: ");
    Serial.println(txPower);
    delay(100);
    IPAddress ip( 192, 168, 1, 1 );
    IPAddress gateway( 192, 168, 1, 1 );
    IPAddress subnet( 255, 255, 255, 0 );
    delay(2000);
    WiFi.softAPConfig( ip, gateway, subnet );  
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    ipAddress = IP.toString();
  }else{
    Serial.print("Setting HapiCap as client to network ");
    Serial.println(config.clientSSID);
    //WiFi.mode(WIFI_STA);
    WiFi.begin(config.clientSSID, config.clientPasswd);
    intCounterWifi = 0;
      
    while (WiFi.status() != WL_CONNECTED){
      delay(500);
      //Serial.print(".");
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