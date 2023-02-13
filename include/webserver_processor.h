// #ifndef WEBSERVER_PROCESSOR_H
// #define WEBSERVER_PROCESSOR_H

// #include <stdint.h>
// #include "Arduino.h"

// String str2HTML;

// String processor(const String& var){
//     if(var == "WPLATLON"){
//           str2HTML = String(config.WAYPOINT_LAT,6) + " " + String(config.WAYPOINT_LON,6);
//           return str2HTML;
//      }else if (var == "GPSTIME"){
//           str2HTML = GPSTime;
//           return str2HTML;            
//      }else if (var == "CURRENTPOSLATLON"){
//           str2HTML = String(flCurrentLat,6) + " " + String(flCurrentLon,6);
//           return str2HTML;
//      }else if (var == "DIST2WP"){
//           str2HTML = String(distancewaypoint);
//           return str2HTML;
//      }else if (var == "WPDIRECTION"){
//           str2HTML = String(cardinalToWaypoint);
//           return str2HTML;
//      }else if (var == "WPBEARING"){
//           str2HTML = String(coarsewaypoint);
//           return str2HTML;
//      }else if (var == "DEGCOMPASS"){
//           str2HTML = String(compassheading);
//           return str2HTML;
//      }else if (var == "CARDCOMPASS"){
//           str2HTML = String(cardinalCompHeading);
//           return str2HTML;
//      }else if (var == "OFFSETCOMPASS"){
//           str2HTML = String(config.compOffset);
//           return str2HTML;
//      }else if (var == "DECLANGLERAD"){
//           str2HTML = String(config.declAngleRad,12);
//           return str2HTML;          
//      }else if (var == "POLLTIMECOMPASS"){
//           str2HTML = String(config.compPollMs);
//           return str2HTML;      
//      }else if (var == "POLLTIMEGPS"){
//           str2HTML = String(config.gpsPollSec);
//           return str2HTML;                 
//      }else if (var == "SERIALCHECK"){
//         if(config.debug2Serial){
//           str2HTML = " checked ";
//         }else{
//           str2HTML = "";
//         }
//         return str2HTML;
//      }
//      /*else if (var == "MPUCHECK"){
//         if(config.Debug4MPU){
//           str2HTML = " checked ";
//         }else{
//           str2HTML = "";
//         }
//         return str2HTML;        
//      }
//      */else if (var == "DATASERIALCHECK"){
//         if(config.debugData2Serial){
//           str2HTML = " checked ";
//         }else{
//           str2HTML = "";  
//         }
//         return str2HTML;                
//      }else if (var == "TELNETCHECK"){
//         if(config.debug2Telnet){
//           str2HTML = " checked ";
//         }else{
//           str2HTML = "";  
//         }
//         return str2HTML;
//      }else if (var == "HAPTICCHECK"){
//         if(config.debugHaptic){
//           str2HTML = " checked ";
//         }else{
//           str2HTML = "";  
//         }
//         return str2HTML;
//      }else if (var == "APCHECK"){
//         if(config.asAP){
//           str2HTML = " checked ";
//         }else{
//           str2HTML = "";
//         }
//         return str2HTML;
//      }else if (var == "TOUCHCHECK"){
//         if(config.touchEnabled){
//           str2HTML = " checked ";
//         }else{
//           str2HTML = "";
//         }
//         return str2HTML;
//      }else if (var == "FTPCHECK"){
//         if(config.ftpEnabled){
//           str2HTML = " checked ";
//         }else{
//           str2HTML = "";
//         }
//         return str2HTML;                                 
//      }else if (var == "NRSAT"){
//         if (nrsatt < 4){
//           str2HTML = String(nrsatt) + " NF";
//         }else{
//           str2HTML = String(nrsatt) + " satellites";
//         }
//         return str2HTML;
//      }else if (var == "STATION"){
//           str2HTML = config.clientSSID;
//           return str2HTML;
//      }else if (var == "PASSWDWIFI"){
//           str2HTML = "**********";
//           return str2HTML;          
//      }else if (var == "MDNSNAME"){
//           str2HTML = String(config.deviceName);
//           return str2HTML;
//      }else if (var == "AP_PASSWD"){
//           str2HTML = String(config.apPasswd);
//           return str2HTML; 
//      }else if (var == "SLEEPTIME"){
//           str2HTML = String(config.sleepMins);
//           return str2HTML; 
//      }else if (var == "TOUCHTHRESHOLD"){
//           str2HTML = String(config.touchThreshold);
//           return str2HTML;
//      }else if (var == "HOME_LAT"){
//           str2HTML = String(config.HOME_LAT,6);
//           return str2HTML;
//      }else if (var == "HOME_LON"){
//           str2HTML = String(config.HOME_LON,6);
//           return str2HTML;
//      }else if (var == "HOMEBEARING"){
//           str2HTML = String(coarse2home);
//           return str2HTML;          
//      }else if (var == "DIST2HOME"){
//           str2HTML = String(distance2home);
//           return str2HTML;
//      }else if (var == "HOMEDIRECTION"){
//           str2HTML = String(cardinal2home);
//           return str2HTML;
//      }else if (var == "TARGETREACHED"){
//           str2HTML = String(config.targetReached);
//           return str2HTML;
//      }else if (var == "MANUALLAT"){
//           str2HTML = String(config.WAYPOINT_LAT,6);
//           return str2HTML;
//      }else if (var == "MANUALLON"){
//           str2HTML = String(config.WAYPOINT_LON,6);
//           return str2HTML;                              
//      }else if (var == "MAXDISTANCEHAPTIC"){
//           str2HTML = String(config.maxDistance);
//           return str2HTML;          
//      }else if (var == "MAXDELAYHAPTIC"){
//           str2HTML = String(config.maxDelay);
//           return str2HTML;          
//      }    
//      /*
//      else if (var == "CALDATAMAGBIASX"){
//           str2HTML = String(caldata.MagBiasX);
//           return str2HTML;
//      }else if (var == "CALDATAMAGBIASY"){
//           str2HTML = String(caldata.MagBiasY);
//           return str2HTML; 
//      }else if (var == "CALDATAMAGBIASZ"){
//           str2HTML = String(caldata.MagBiasZ);
//           return str2HTML;
//      }else if (var == "CALDATAMAGSCALEX"){
//           str2HTML = String(caldata.MagScaleFacX);
//           return str2HTML; 
//      }else if (var == "CALDATAMAGSCALEY"){
//           str2HTML = String(caldata.MagScaleFacY);
//           return str2HTML; 
//      }else if (var == "CALDATAMAGSCALEZ"){
//           str2HTML = String(caldata.MagScaleFacZ);
//           return str2HTML;
//      }else if (var == "CALDATAGYROBIASX"){
//           str2HTML = String(caldata.GyroBiasX);
//           return str2HTML;
//      }else if (var == "CALDATAGYROBIASY"){
//           str2HTML = String(caldata.GyroBiasY);
//           return str2HTML;  
//      }else if (var == "CALDATAGYROBIASZ"){
//           str2HTML = String(caldata.GyroBiasZ);
//           return str2HTML;
//      }else if (var == "CALDATAACCELBIASX"){
//           str2HTML = String(caldata.AccelBiasX);
//           return str2HTML;
//      }else if (var == "CALDATAACCELBIASY"){
//           str2HTML = String(caldata.AccelBiasY);
//           return str2HTML; 
//      }else if (var == "CALDATAACCELBIASZ"){
//           str2HTML = String(caldata.AccelBiasZ);
//           return str2HTML;
//      }else if (var == "CALDATAACCELSCALEX"){
//           str2HTML = String(caldata.AccelScaleX);
//           return str2HTML; 
//      }else if (var == "CALDATAACCELSCALEY"){
//           str2HTML = String(caldata.AccelScaleY);
//           return str2HTML; 
//      }else if (var == "CALDATAACCELSCALEZ"){
//           str2HTML = String(caldata.AccelScaleZ);
//           return str2HTML;                      
//      }else if (var == "TEMPERATURE"){
//           str2HTML = String(flTemperature);
//           return str2HTML;                                  
//      }
//      */                    
//     return String();
// }


// #endif