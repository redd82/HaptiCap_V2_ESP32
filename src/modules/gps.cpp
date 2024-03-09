#include "gps.h"

extern TinyGPSPlus gps;
extern int hours;
extern int mins;
extern int secs;
extern String GPSTimeMinsSecs;
extern String GPSTimeMins;
extern String GPSDate;
extern int day;
extern int month;
extern int year;
extern int GPSFix;
extern int GPSFixAccepted;

extern Config config;
extern SensorData sensorData;

// GPS functions
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