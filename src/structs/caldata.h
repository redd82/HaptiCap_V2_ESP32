#ifndef CALDATA_H
#define CALDATA_H

struct CalData {
  bool compassCalibrated = false;
  float compassOffset = 0;
  int16_t magOffsetX = 0;     // Mag_x_scale = 1.01,  
  int16_t magOffsetY = 0;     // Mag_y_scale = 0.99,
  int16_t magOffsetZ = 0;     // Mag_z_scale = 1.00;
  int16_t magRadius = 0;
  int16_t gyroOffsetX = 0;
  int16_t gyroOffsetY = 0;
  int16_t gyroOffsetZ = 0;
  int16_t accelOffsetX = 0;
  int16_t accelOffsetY = 0;
  int16_t accelOffsetZ = 0;
  int16_t accelRadius = 0;
  int16_t system = 0;
};

#endif
