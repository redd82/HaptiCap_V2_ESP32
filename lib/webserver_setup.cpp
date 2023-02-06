#include <Arduino.h>
// Webweberver setup responses
  
  webServer.on("/", HTTP_GET, [](AsyncWebwebServerRequest *request){
  request->send(SPIFFS, "/areamap.html", String(), false, processor);
  timerRestart(timer2);
  if(config.Debug2Serial){
    Serial.println("areamap called");
  }
});

  webServer.on("/areamap.html", HTTP_GET, [](AsyncWebwebServerRequest *request){
  request->send(SPIFFS, "/areamap.html", String(), false, processor);
  timerRestart(timer2);
  if(config.Debug2Serial){
    Serial.println("areamap called");
  }
});

  webServer.on("/areamap_test.html", HTTP_GET, [](AsyncWebwebServerRequest *request){
  request->send(SPIFFS, "/areamap_test.html", String(), false, processor);
  timerRestart(timer2);
  if(config.Debug2Serial){  
    Serial.println("areamap called");
  }
});

  webServer.on("/style.css", HTTP_GET, [](AsyncWebwebServerRequest *request){
  request->send(SPIFFS, "/style.css", "text/css");
});

  webServer.on("/map.jpg", HTTP_GET, [](AsyncWebwebServerRequest *request){
  request->send(SPIFFS, "/map.jpg", "image/jpeg");
});

  webServer.on("/map_test.jpg", HTTP_GET, [](AsyncWebwebServerRequest *request){
  request->send(SPIFFS, "/map_test.jpg", "image/jpeg");
});

  webServer.on("/manifest.txt", HTTP_GET, [](AsyncWebwebServerRequest *request){
  request->send(SPIFFS, "/manifest.txt", "text/css");  
});

// Send a GET request to <ESP_IP>/get?input1=<inputMessage> set.html?lat=123.456&lon=78.90  setlatlong?lat=123.456&lon=78.90
weberver.on("/setlatlon", HTTP_GET, [] (AsyncWebweberverRequest *request){
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
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
    if(config.Debug2Serial){
      Serial.println("WP Set");
      Serial.println(config.WAYPOINT_LAT,6);
      Serial.println(config.WAYPOINT_LON,6);
    }    
    saveConfiguration(filename, config);    
    request->send(SPIFFS, "/areamap.html", String(), false, processor);
    timerRestart(timer2);    
  });

weberver.on("/setlatlon_home", HTTP_GET, [] (AsyncWebweberverRequest *request){
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    flCurrentLat = gps.location.lat();
    flCurrentLon = gps.location.lng();
    config.HOME_LAT = flCurrentLat;
    config.HOME_LON = flCurrentLon;
    coarse2home = coarse2waypoint(config.HOME_LAT, config.HOME_LON);
    distance2home = distance2waypoint(config.HOME_LAT, config.HOME_LON);
    cardinal2home = TinyGPSPlus::cardinal(coarse2home);
    nrsatt = gps.satellites.value();
    if(config.Debug2Serial){
      Serial.println("Home Set");
      Serial.println(config.HOME_LAT,6);
      Serial.println(config.HOME_LON,6);
    }
    saveConfiguration(filename, config);
    request->send(SPIFFS, "/areamap.html", String(), false, processor);
    timerRestart(timer2);    
  });    

weberver.on("/setlatlon_home_test", HTTP_GET, [] (AsyncWebweberverRequest *request){
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    flCurrentLat = gps.location.lat();
    flCurrentLon = gps.location.lng();
    config.HOME_LAT = flCurrentLat;
    config.HOME_LON = flCurrentLon;
    coarse2home = coarse2waypoint(config.HOME_LAT, config.HOME_LON);
    distance2home = distance2waypoint(config.HOME_LAT, config.HOME_LON);
    cardinal2home = TinyGPSPlus::cardinal(coarse2home);
    nrsatt = gps.satellites.value();        
    if(config.Debug2Serial){
      Serial.println("Home Set");
      Serial.println(config.HOME_LAT,6);
      Serial.println(config.HOME_LON,6);
    }
    saveConfiguration(filename, config);
    request->send(SPIFFS, "/areamap_test.html", String(), false, processor);
    timerRestart(timer2);    
  });  
    
weberver.on("/setlatlon_test", HTTP_GET, [] (AsyncWebweberverRequest *request){
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
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
    if(config.Debug2Serial){
      Serial.println("WP Set");
      Serial.println(config.WAYPOINT_LAT,6);
      Serial.println(config.WAYPOINT_LON,6);
    }    
    saveConfiguration(filename, config);   
    request->send(SPIFFS, "/areamap_test.html", String(), false, processor);
    timerRestart(timer2);    
  });

weberver.on("/settings.html", HTTP_GET, [] (AsyncWebweberverRequest *request){
  if (request->hasParam(PARAM_GENERALAPPLY)){
    if (request->hasParam(PARAM_DEBUGSERIAL)){
         inputSetting = request->getParam(PARAM_DEBUGSERIAL)->value();
              if(inputSetting == "debug2serial"){
              config.Debug2Serial = 1;
              Serial.println("config.Debug2Serial = 1;"); 
             }
    }else{
      config.Debug2Serial = 0;
      Serial.println("config.Debug2Serial = 0;");
    }    
    if (request->hasParam(PARAM_DEBUGTELNET)) {
        inputSetting = request->getParam(PARAM_DEBUGTELNET)->value();
          if(inputSetting == "telnet"){
              config.Debug2Telnet = 1;
              if(config.Debug2Serial){
                Serial.println("config.Debug2Telnet = 1;"); 
              }
          }
    }else{
      config.Debug2Telnet = 0;
      if(config.Debug2Serial){
        Serial.println("config.Debug2Telnet = 0;");             
      }
    }
    if (request->hasParam(PARAM_DEBUGDATASERIAL)) {
         inputSetting = request->getParam(PARAM_DEBUGDATASERIAL)->value();
              if(inputSetting == "Dataserial"){
              config.DebugData2Serial = 1;
              if(config.Debug2Serial){
                Serial.println("config.DebugData2Serial = 1;"); 
              }
              bPrintHeader = 1;
          }
    }else{
      config.DebugData2Serial = 0;
      if(config.Debug2Serial){
        Serial.println("config.DebugData2Serial = 0;");
      }
          }              
    if (request->hasParam(PARAM_DEBUGHAPTIC)) {
        inputSetting = request->getParam(PARAM_DEBUGHAPTIC)->value();      
          if(inputSetting == "haptic"){
              config.DebugHaptic = 1;
              if(config.Debug2Serial){   
              Serial.println("config.DebugHaptic = 1;");      
              }
          }     
    }else{
      config.DebugHaptic = 0;
      if(config.Debug2Serial){
        Serial.println("config.DebugHaptic = 0;"); 
      }
    }
    /*
    if (request->hasParam(PARAM_DEBUGMPU)) {
        inputSetting = request->getParam(PARAM_DEBUGMPU)->value();      
          if(inputSetting == "debug4mpu"){
              config.Debug4MPU = 1;
              if(config.Debug2Serial){   
                Serial.println("config.Debug4MPU = 1;");      
              }
          }     
    }else{
      config.Debug4MPU = 0;
      if(config.Debug2Serial){
        Serial.println("config.Debug4MPU = 0;"); 
      }
    }  */  
    if (request->hasParam(PARAM_FTPMODE)) {
        inputSetting = request->getParam(PARAM_FTPMODE)->value();      
          if(inputSetting == "ftp"){
              config.FTPEnabled = 1;
              if(config.FTPEnabled){   
              Serial.println("config.FTPEnabled = 1;");      
              }
          }     
    }else{
      config.FTPEnabled = 0;
      if(config.FTPEnabled){
        Serial.println("config.FTPEnabled = 0;"); 
      }
    }    
    if(request->hasParam(PARAM_SSID)){
        inputSetting = request->getParam(PARAM_SSID)->value();
        if(inputSetting.length() > 0 && inputSetting.length() < 17){
          inputSetting.toCharArray(config.Client_SSID,16);
          if(config.Debug2Serial){
            Serial.println(inputSetting);
          }
          bSaveConfig = 1;
        }else{
          if(config.Debug2Serial){
          Serial.println(config.Client_SSID);    
          }
        }
          if (request->hasParam(PARAM_APMODE)) {
          inputSetting = request->getParam(PARAM_APMODE)->value();      
            if(inputSetting == "ap_modeon"){
              if(!config.As_AP){
                config.As_AP = 1;
                if(config.Debug2Serial){
                  Serial.println("ap_mode_onoff =>1");
                }
                saveConfiguration(filename, config);
                delay(1000);
                ESP.restart();      
              }
          }     
            }else{
              if(config.As_AP){
                config.As_AP = 0;
                if(config.Debug2Serial){                
                  Serial.println("ap_mode_onoff =>0");
                }
                saveConfiguration(filename, config);
                delay(1000);
                ESP.restart();
              }  
            }
          }
          if(request->hasParam(PARAM_CLIENTPASSWD)){
              inputSetting = request->getParam(PARAM_CLIENTPASSWD)->value();
              if(inputSetting.length() > 0 && inputSetting.length() < 25){
                if(inputSetting == "**********"){
                   if(config.Debug2Serial){
                    Serial.println("**********");
                   }
                }else{
                  inputSetting.toCharArray(config.Client_Passwd,24);
                  if(config.Debug2Serial){
                    Serial.println(inputSetting);
                  }              
                  bSaveConfig = 1;
                  if(config.Debug2Serial){
                    Serial.println(config.Client_Passwd);
                  }
                }
              }
          }
          if(request->hasParam(PARAM_DEVICENAME)){
              inputSetting = request->getParam(PARAM_DEVICENAME)->value();
              if(inputSetting.length() > 0  && inputSetting.length() < 25){
                inputSetting.toCharArray(config.DeviceName,24);
                if(config.Debug2Serial){
                  Serial.println(inputSetting);
                }
                bSaveConfig = 1;
                if(config.Debug2Serial){
                  Serial.println(config.DeviceName);
                }
              }
          }
        if(request->hasParam(PARAM_AP_PASSWD)){
            inputSetting = request->getParam(PARAM_AP_PASSWD)->value();
            if(inputSetting.length() > 0 && inputSetting.length() < 25){
              inputSetting.toCharArray(config.AP_Passwd,24);
              if(config.Debug2Serial){
                Serial.println(inputSetting);
              }
              bSaveConfig = 1;
              if(config.Debug2Serial){
                Serial.println(config.AP_Passwd);
              }
            }
        }
     
        if(request->hasParam(PARAM_COMPASSOFFSET)){
            inputSetting = request->getParam(PARAM_COMPASSOFFSET)->value();
            if(inputSetting.length() > 0  && inputSetting.length() < 10){
              config.Comp_offset = inputSetting.toFloat();
              if(config.Debug2Serial){
                Serial.println(inputSetting);
              }
              bSaveConfig = 1;
              if(config.Debug2Serial){
                Serial.println(config.Comp_offset);
              }
            }
        }
        if(request->hasParam(PARAM_COMPASSDECLANGLE)){
            inputSetting = request->getParam(PARAM_COMPASSDECLANGLE)->value();
            if(inputSetting.length() > 0  && inputSetting.length() < 15){
              config.DeclAngleRad = inputSetting.toDouble();
              if(config.Debug2Serial){
                Serial.println(inputSetting);
              }
              bSaveConfig = 1;
              if(config.Debug2Serial){
                Serial.println(config.DeclAngleRad);
              }
            }
        }    
        if(request->hasParam(PARAM_COMPASSPOLLTIME)){
            inputSetting = request->getParam(PARAM_COMPASSPOLLTIME)->value();
              if(inputSetting.length() > 0  && inputSetting.length() < 5){
                if(config.Debug2Serial){
                  Serial.println(inputSetting);
                }
                config.Comp_poll_ms = inputSetting.toInt();
                bSaveConfig = 1;
                timerEnd(timer1);
                timer1 = timerBegin(1, 80, true);
                timerAttachInterrupt(timer1, &onTimer1, true);
                timerAlarmWrite(timer1, (config.Comp_poll_ms * 1000), true);            // 1 ms
                timerAlarmEnable(timer1);
                timerRestart(timer1);              
                if(config.Debug2Serial){
                  Serial.println(config.Comp_poll_ms);                      
                }
              }
        }
   
        if(request->hasParam(PARAM_GPSPOLLTIME)){
            inputSetting = request->getParam(PARAM_GPSPOLLTIME)->value();
              if(inputSetting.length() > 0  && inputSetting.length() < 5){
                if(config.Debug2Serial){
                  Serial.println(inputSetting);
                }
                config.GPS_poll_sec = inputSetting.toInt();
                bSaveConfig = 1;
                timerEnd(timer0);
                timer0 = timerBegin(0, 80, true);
                timerAttachInterrupt(timer0, &onTimer0, true);
                timerAlarmWrite(timer0, (config.GPS_poll_sec * 1000000), true);             // sec
                timerAlarmEnable(timer0);
                timerRestart(timer0);
                if(config.Debug2Serial){
                  Serial.println(config.GPS_poll_sec);            
                }
              }
        }
        if(request->hasParam(PARAM_GPSTARGETREACHED)){
            inputSetting = request->getParam(PARAM_GPSTARGETREACHED)->value();
              if(inputSetting.length() > 0  && inputSetting.length() < 3){
                if(config.Debug2Serial){
                  Serial.println(inputSetting);
                }
                config.intTargetReached = inputSetting.toInt();
                bSaveConfig = 1;
                if(config.Debug2Serial){
                  Serial.println(config.intTargetReached);            
                }
              }
        }
        if(request->hasParam(PARAM_LAT)){
            inputSetting = request->getParam(PARAM_LAT)->value();
              if(inputSetting.length() > 0  && inputSetting.length() < 10){
                if(config.Debug2Serial){
                  Serial.println(inputSetting);
                }
                config.WAYPOINT_LAT = inputSetting.toDouble();
                bSaveConfig = 1;
                if(config.Debug2Serial){
                  Serial.println(config.WAYPOINT_LAT);            
                }
              }
        }  
        if(request->hasParam(PARAM_LON)){
            inputSetting = request->getParam(PARAM_LON)->value();
              if(inputSetting.length() > 0  && inputSetting.length() < 10){
                if(config.Debug2Serial){
                  Serial.println(inputSetting);
                }
                config.WAYPOINT_LON = inputSetting.toDouble();
                bSaveConfig = 1;
                if(config.Debug2Serial){
                  Serial.println(config.WAYPOINT_LON);            
                }
              }
        }                          
        if(request->hasParam(PARAM_ESP_SLEEPTIME)){
            inputSetting = request->getParam(PARAM_ESP_SLEEPTIME)->value();
              if(inputSetting.length() > 0  && inputSetting.length() < 5){
                if(config.Debug2Serial){
                  Serial.println(inputSetting);
                }
                config.Sleep_mins = inputSetting.toInt();
                bSaveConfig = 1;
                timerEnd(timer2);
                timer2 = timerBegin(2, 80, true);
                timerAttachInterrupt(timer2, &onTimer2, true);                
                timerAlarmWrite(timer2, (config.Sleep_mins * 60000000), true);            // mins
                timerAlarmEnable(timer2);
                timerRestart(timer2);
                if(config.Debug2Serial){
                  Serial.println(config.Sleep_mins); 
                }
              }
        }
    
        if (request->hasParam(PARAM_TOUCH)) {
            inputSetting = request->getParam(PARAM_TOUCH)->value();      
              if(inputSetting == "touch_enabled"){
                  config.TouchEnabled = 1;
                  bSaveConfig = 1;
                  if(config.Debug2Serial){   
                    Serial.println("config.TouchEnabled = 1;");      
                  }
              }     
        }else{
          bSaveConfig = 1;
          config.TouchEnabled = 0;
          if(config.Debug2Serial){
            Serial.println("config.TouchEnabled = 0;"); 
          }
        }    
        if(request->hasParam(PARAM_TOUCHTHRESHOLD)){
            inputSetting = request->getParam(PARAM_TOUCHTHRESHOLD)->value();
              if(inputSetting.length() > 0  && inputSetting.length() < 3){
                config.Touch_Threshold = inputSetting.toInt();
                bSaveConfig = 1;
                timerRestart(timer2);
                if(config.Debug2Serial){
                  Serial.println(config.Touch_Threshold); 
                }
              }
        }
        if(request->hasParam(PARAM_MAXDELAY)){
            inputSetting = request->getParam(PARAM_MAXDELAY)->value();
              if(inputSetting.length() > 0  && inputSetting.length() < 5){
                config.maxdelay = inputSetting.toInt();
                if(config.maxdelay < 200){
                  config.maxdelay = 200;
                }
                 
                bSaveConfig = 1;
                timerRestart(timer2);
                if(config.Debug2Serial){
                  Serial.println(config.maxdelay); 
                }
              }
        }   
        if(request->hasParam(PARAM_MAXDISTANCE)){
            inputSetting = request->getParam(PARAM_MAXDISTANCE)->value();
              if(inputSetting.length() > 0  && inputSetting.length() < 5){
                config.maxdistance = inputSetting.toInt();
                if(config.maxdistance < 100) {
                  config.maxdistance = 100;
                }
                bSaveConfig = 1;
                timerRestart(timer2);
                if(config.Debug2Serial){
                  Serial.println(config.maxdistance); 
                }
              }
        }
                      
    }else if (request->hasParam(PARAM_GENERALSETNORTH)){
         inputSetting = request->getParam(PARAM_GENERALSETNORTH)->value();
              if(inputSetting == "SetNorth"){
                if(headingraw <= 180){
                  config.Comp_offset = headingraw;
                }
                if(headingraw > 180){
                config.Comp_offset = -1.0 * (360 - headingraw);
                }
                if(config.Debug2Serial){
                  Serial.print("Heading compass raw: "); 
                  Serial.println(headingraw); 
                  Serial.print("Heading Offset: "); 
                  Serial.println(config.Comp_offset); 
                }
             bSaveConfig = 1;                                   
             }
    } 
    /*
    else if (request->hasParam(PARAM_GENERALCALGYRO)){
         inputSetting = request->getParam(PARAM_GENERALCALGYRO)->value();
              if(inputSetting == "Gyro"){
                bCalGyro = 1;
             }      
          
    }else if (request->hasParam(PARAM_GENERALCALACCEL)){
         inputSetting = request->getParam(PARAM_GENERALCALACCEL)->value();
              if(inputSetting == "Accelerometer"){
                bCalAccel = 1;
             }
      
    }else if (request->hasParam(PARAM_GENERALCALMAG)){
         inputSetting = request->getParam(PARAM_GENERALCALMAG)->value();
              if(inputSetting == "Compass"){
                bCalMag = 1;
             }
      
    }
    */
    if(bSaveConfig){
    saveConfiguration(filename, config);         
    bSaveConfig = 0;
    }
    request->send(SPIFFS, "/settings.html",  String(), false, processor);
    timerRestart(timer2);
    if(config.Debug2Serial){
      Serial.println("Settings.html Called");
    }
  });

/*
weberver.on("/cal_data.html", HTTP_GET, [] (AsyncWebweberverRequest *request){
    if (request->hasParam(PARAM_GENERALCALGYRO)){
         inputSetting = request->getParam(PARAM_GENERALCALGYRO)->value();
              if(inputSetting == "Gyro"){
                bCalGyro = 1;
                  if(config.Debug2Serial){
                    Serial.println("bCalGyro = 1");
                  }
              }    
          }
     if (request->hasParam(PARAM_GENERALCALACCEL)){
         inputSetting = request->getParam(PARAM_GENERALCALACCEL)->value();
              if(inputSetting == "Accelerometer"){
                bCalAccel = 1;
                  if(config.Debug2Serial){
                    Serial.println("bCalAccel = 1");
                  }                
             }
          }
   if (request->hasParam(PARAM_GENERALCALMAG)){
         inputSetting = request->getParam(PARAM_GENERALCALMAG)->value();
              if(inputSetting == "Compass"){
                bCalMag = 1;
                  if(config.Debug2Serial){
                    Serial.println("bCalMag = 1");
                  }                
             }
          }
    request->send(SPIFFS, "/cal_data.html",  String(), false, processor);
    timerRestart(timer2);
    if(config.Debug2Serial){
      Serial.println("cal_data.html Called");
    }  
});
  */
  webServer.onNotFound(notFound);
  webServer.begin();