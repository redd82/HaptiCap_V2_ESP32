// #include "compass.h"

// extern uint16_t BNO055_SAMPLERATE_DELAY_MS;
// extern Adafruit_BNO055 bno;
// extern SF fusion;
// extern double DEG_2_RAD;
// extern float deltat;

// float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
// float pitch, roll, yaw;
// float deltat;

// // ----- Record compass offsets, scale factors, & ASA values
// /*
//    These values seldom change ... an occasional check is sufficient
//    (1) Open your Arduino "Serial Monitor
//    (2) Set "Record_data=true;" then upload & run program.
//    (3) Replace the values below with the values that appear on the Serial Monitor.
//    (4) Set "Record_data = false;" then upload & rerun program.
// */
// // bool    Record_data = true;
// // int     Mag_x_offset = 46,      Mag_y_offset = 190,     Mag_z_offset = -254;   // Hard-iron offsets
// // float   Mag_x_scale = 1.01,     Mag_y_scale = 0.99,     Mag_z_scale = 1.00;    // Soft-iron scale factors
// // float   ASAX = 1.17,            ASAY = 1.18,            ASAZ = 1.14;           // (A)sahi (S)ensitivity (A)djustment fuse ROM values.

// // Set declination angle on your location and fix heading
// // Formula: (deg + (minutes / 60.0)) / (180 / M_PI); (4.0 + (26.0 / 60.0)) / (180 / PI);
// //float declinationAngle = (declAngleDeg + (declAngleMin / 60.0)) / (180.0 / PI);

// void magnometerSetup(){
//   // start communication with IMU 
//   if (!bno.begin())
//   {
//     Serial.print("No BNO055 detected");
//     while (1);
//   }

// }

// double getCompassHeading(){
//   double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
//   double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
//   double xPos = 0, yPos = 0, headingVel = 0;
//   sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
//   bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
//   bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
//   bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
//   bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
//   bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
//   bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

//   xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
//   yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;
//   headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

//   // ax = IMU.getAccelX_mss();
//   // ay = IMU.getAccelY_mss();
//   // az = IMU.getAccelZ_mss();
//   // gx = IMU.getGyroX_rads();
//   // gy = IMU.getGyroY_rads();
//   // gz = IMU.getGyroZ_rads();
//   // mx = IMU.getMagX_uT();
//   // my = IMU.getMagY_uT();
//   // mz = IMU.getMagZ_uT();
//   // temp = IMU.getTemperature_C();
//   deltat = fusion.deltatUpdate();
//   //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //mahony is suggested if there isn't the mag
//   //fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick
  
  
//   double x = magnetometerData.magnetic.x;
//  // Serial.println(x);
//   //roll = fusion.getRoll();
//   // = fusion.getPitch();
//   // = fusion.getYaw();
//   //return yaw;
//   return headingVel;
// }

// // float getCompassHeading(){
// //   float sum = 0.0;
// //   xyzFloat gValue = myMPU6500.getGValues();
// //   xyzFloat gyr = myMPU6500.getGyrValues();
// //   xyzFloat magValue = myMPU6500.getMagValues();
// //   float temp = myMPU6500.getTemperature();
// //   float resultantG = myMPU6500.getResultantG(gValue);
// //   previous_compassheading = compassheading;
// //   // Calculate heading
// //   float heading = atan2(magValue.y, magValue.x);
// //   headingraw = heading;
// //   //float heading = atan2(event.magnetic.y, event.magnetic.x);
// //   float declinationAngle = config.declAngleRad;
// //   heading += declinationAngle;
  
// //   // Correct for heading < 0deg and heading > 360deg
// //   if (heading < 0){
// //     heading += 2.0 * PI;
// //   }

// //   if (heading > 2.0 * PI){
// //     heading -= 2.0 * PI;
// //   }
// //   // Convert to degrees
// //   float headingDegrees = heading * 180.0/M_PI;
// //   if(caldata.compassOffset >= 0){
// //     if(headingDegrees < caldata.compassOffset){
// //       headingDegrees = headingDegrees - caldata.compassOffset + 360.0;
// //     }else{
// //       headingDegrees = headingDegrees - caldata.compassOffset;
// //     }
// //   }else{
// //       headingDegrees = headingDegrees + (-1.0 * caldata.compassOffset);
// //       if(headingDegrees >= 360)
// //       {
// //           headingDegrees = headingDegrees - 360;
// //       }
// //   }
// //   return headingDegrees;
// // }