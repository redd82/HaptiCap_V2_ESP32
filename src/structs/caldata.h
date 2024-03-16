#ifndef CALDATA_H
#define CALDATA_H

struct CalData {
  bool compassCalibrated = false;
  float compassOffset = 0;
  float magOffsetX = 0;     // uT
  float magOffsetY = 0;     
  float magOffsetZ = 0;  
  float magSoftIron[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};   
  float magRadius = 0;
  float magField = 0;       // The magnetic field magnitude in uTesla
  float gyroOffsetX = 0;    // rad/s
  float gyroOffsetY = 0;
  float gyroOffsetZ = 0;
  float accelOffsetX = 0;   // m/s^2
  float accelOffsetY = 0;
  float accelOffsetZ = 0;
  float accelRadius = 0;

  int16_t accelDone = 0;
  int16_t gyroDone = 0;
  int16_t magDone = 0;
  int16_t system = 0;
};

#endif
