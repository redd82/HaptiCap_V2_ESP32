#ifndef CALDATA_H
#define CALDATA_H

struct CalData {
  bool calibrateMag = false;
  float compassOffset = 0.0;
  int magBiasX = 0;             // Mag_x_offset = 46,
  int magBiasY = 0;             // Mag_y_offset = 190,
  int magBiasZ = 0;             // Mag_z_offset = -254;
  float magScaleFacX = 1.0;     // Mag_x_scale = 1.01,  
  float magScaleFacY = 1.0;     // Mag_y_scale = 0.99,
  float magScaleFacZ = 1.0;     // Mag_z_scale = 1.00;
  float ASAX = 1.0;             // (A)sahi (S)ensitivity (A)djustment fuse ROM values.
  float ASAY = 1.0;
  float ASAZ = 1.0;
  float gyroBiasX = 0.0;
  float gyroBiasY = 0.0;
  float gyroBiasZ = 0.0;
  float accelBiasX = 0.0;
  float accelBiasY = 0.0;
  float accelBiasZ = 0.0;
  float accelScaleX = 1.0;
  float accelScaleY = 1.0;
  float accelScaleZ = 1.0;
};

#endif
