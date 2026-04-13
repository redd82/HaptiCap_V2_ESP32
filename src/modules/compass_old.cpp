// #include "compass.h"


// extern uint16_t BNO055_SAMPLERATE_DELAY_MS;
// extern SF fusion;

// extern Adafruit_LIS3MDL lis3mdl;
// extern Adafruit_LSM6DSOX sox;

// Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
// sensors_event_t mag_event, gyro_event, accel_event;

// extern CalData caldata;
// extern double DEG_2_RAD;
// extern float deltat;

// extern bool magnetometerCalibrated;

// extern bool compassCalibrated;
// extern float compassheading;

// int loopcount = 0;

// byte calibrationData[68]; // buffer to receive magnetic calibration data
// byte calcount=0;

// float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
// float pitch, roll, yaw;
// float deltat;

// float thetaM;
// float phiM;
// float thetaFold=0;
// float thetaFnew;
// float phiFold=0;
// float phiFnew;
// float thetaG=0;
// float phiG=0;
// float theta;
// float phi;
// float thetaRad;
// float phiRad;
 
// float Xm;
// float Ym;
// float psi;

// float dt;
// unsigned long millisOld;

// void setup_sensors(void) {

// }

// void setupLis3mdlLSM6DSOX(){
//   if (! lis3mdl.begin_I2C()) {          
//     // hardware I2C mode, can pass in address & alt Wire
//     //if (! lis3mdl.begin_SPI(LIS3MDL_CS)) {  // hardware SPI mode
//     //if (! lis3mdl.begin_SPI(LIS3MDL_CS, LIS3MDL_CLK, LIS3MDL_MISO, LIS3MDL_MOSI)) { // soft SPI
//     Serial.println("Failed to find LIS3MDL chip");
//     while (1) { delay(10); }
//   }

//   if (!sox.begin_I2C()) {
//     // if (!sox.begin_SPI(LSM_CS)) {
//     // if (!sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
//     Serial.println("Failed to find LSM6DSOX chip");
//     while (1) {
//       delay(10);
//     }
//   }
//   accelerometer->printSensorDetails();
//   gyroscope->printSensorDetails();
//   magnetometer->printSensorDetails();

//   // set lowest range
//   sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
//   sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
//   lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

//   // set slightly above refresh rate
//   sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
//   sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
//   lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
//   lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE); 
//   lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

//   accelerometer = sox.getAccelerometerSensor();
//   gyroscope = sox.getGyroSensor();
//   magnetometer = &lis3mdl;
//   Wire.setClock(400000); // 400KHz
// }

// void compassSetup(){
//   Serial.println("Compass setup...");
//     void setupLis3mdlLSM6DSOX();


//     if(caldata.compassCalibrated){
//       Serial.println("Compass already calibrated");
      
//       //adafruit_bno055_offsets_t calibrationData;            // this is a struct of data standardized for bno055
//       // calibrationData.accel_offset_x = caldata.accelOffsetX;
//       // calibrationData.accel_offset_y = caldata.accelOffsetY;
//       // calibrationData.accel_offset_z = caldata.accelOffsetZ;
//       // calibrationData.accel_radius = caldata.accelRadius;
//       // calibrationData.gyro_offset_x = caldata.gyroOffsetX;
//       // calibrationData.gyro_offset_y = caldata.gyroOffsetY;
//       // calibrationData.gyro_offset_z = caldata.gyroOffsetZ;
//       // calibrationData.mag_offset_x = caldata.magOffsetX;
//       // calibrationData.mag_offset_y = caldata.magOffsetY;
//       // calibrationData.mag_offset_z = caldata.magOffsetZ;
//       // calibrationData.mag_radius = caldata.magRadius;
//       // bno.setSensorOffsets(calibrationData);
//     }else{
//       Serial.println("Compass not calibrated");
//     }
// }

// // void readCalData(){
// // // select either EEPROM or SPI FLASH storage:
// //   Adafruit_Sensor_Calibration_EEPROM cal;

// //   void setupCalStore() {
// //     delay(100);
// //     Serial.println("Calibration filesys test");
// //     if (!cal.begin()) {
// //       Serial.println("Failed to initialize calibration helper");
// //       while (1) yield();
// //     }
// //     Serial.print("Has EEPROM: "); Serial.println(cal.hasEEPROM());
// //     Serial.print("Has FLASH: "); Serial.println(cal.hasFLASH());

// //     if (! cal.loadCalibration()) {
// //       Serial.println("No calibration loaded/found... will start with defaults");
// //     } else {
// //       Serial.println("Loaded existing calibration");
// //     }

// //     // in uTesla
// //     cal.mag_hardiron[0] = -3.35;
// //     cal.mag_hardiron[1] = -0.74;
// //     cal.mag_hardiron[2] = -40.79;

// //     // in uTesla
// //     cal.mag_softiron[0] = 0.965;
// //     cal.mag_softiron[1] = 0.018;
// //     cal.mag_softiron[2] = 0.010;  
// //     cal.mag_softiron[3] = 0.018;
// //     cal.mag_softiron[4] = 0.960;
// //     cal.mag_softiron[5] = 0.003;  
// //     cal.mag_softiron[6] = 0.010;
// //     cal.mag_softiron[7] = 0.003;
// //     cal.mag_softiron[8] = 1.080;
// //     // Earth total magnetic field strength in uTesla (dependent on location and time of the year),
// //     // visit: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm)
// //     cal.mag_field = 45.00; // approximate value for locations along the equator

// //     // in Radians/s
// //     cal.gyro_zerorate[0] = 0.05;
// //     cal.gyro_zerorate[1] = -0.01;
// //     cal.gyro_zerorate[2] = -0.01;

// //     if (! cal.saveCalibration()) {
// //       Serial.println("**WARNING** Couldn't save calibration");
// //     } else {
// //       Serial.println("Wrote calibration");    
// //     }

// //     cal.printSavedCalibration();
// //   }
// // }

// void displaySensorDetails(void)
// {
//   accelerometer->printSensorDetails();
//   gyroscope->printSensorDetails();
//   magnetometer->printSensorDetails();
//   delay(500);
// }

// void displaySensorStatus(void)
// {
//     /* Get the system status values (mostly for debugging purposes) */
//     // uint8_t system_status, self_test_results, system_error;
//     // system_status = self_test_results = system_error = 0;
//     // bno.getSystemStatus(&system_status, &self_test_results, &system_error);

//     /* Display the results in the Serial Monitor */
//     // Serial.println("");
//     // Serial.print("System Status: 0x");
//     // Serial.println(system_status, HEX);
//     // Serial.print("Self Test:     0x");
//     // Serial.println(self_test_results, HEX);
//     // Serial.print("System Error:  0x");
//     // Serial.println(system_error, HEX);
//     // Serial.println("");
//     // delay(500);
// }

// void displayCalStatus(void)
// {
//     /* Get the four calibration values (0..3) */
//     /* Any sensor data reporting 0 should be ignored, */
//     /* 3 means 'fully calibrated" */
//     // uint8_t system, gyro, accel, mag;
//     // system = gyro = accel = mag = 0;
//     // bno.getCalibration(&system, &gyro, &accel, &mag);

//     // /* The data should be ignored until the system calibration is > 0 */
//     // Serial.print("\t");
//     // if (!system)
//     // {
//     //     Serial.print("! ");
//     // }

//     /* Display the individual values */
//     // Serial.print("Sys:");
//     // Serial.print(system, DEC);
//     // Serial.print(" G:");
//     // Serial.print(gyro, DEC);
//     // Serial.print(" A:");
//     // Serial.print(accel, DEC);
//     // Serial.print(" M:");
//     // Serial.print(mag, DEC);
// }

// void displaySensorOffsets(CalData &caldata)
// {
//     Serial.print("Accelerometer: ");
//     Serial.print(caldata.accelOffsetX); Serial.print(" ");
//     Serial.print(caldata.accelOffsetY); Serial.print(" ");
//     Serial.print(caldata.accelOffsetZ); Serial.print(" ");

//     Serial.print("\nGyro: ");
//     Serial.print(caldata.gyroOffsetX); Serial.print(" ");
//     Serial.print(caldata.gyroOffsetY); Serial.print(" ");
//     Serial.print(caldata.gyroOffsetZ); Serial.print(" ");

//     Serial.print("\nMag: ");
//     Serial.print(caldata.magOffsetX); Serial.print(" ");
//     Serial.print(caldata.magOffsetY); Serial.print(" ");
//     Serial.print(caldata.magOffsetZ); Serial.print(" ");

//     Serial.print("\nAccel Radius: ");
//     Serial.print(caldata.accelRadius);

//     Serial.print("\nMag Radius: ");
//     Serial.print(caldata.magRadius);
// }

// void calibrateCompass(){
//     Serial.println("Calibrating compass...");
//   while(!caldata.compassCalibrated){
//     uint8_t system, gyro, accel, mg = 0;
// ;
//     if (system < 3 || gyro < 3 || accel < 3 || mg < 3) {
//         caldata.compassCalibrated   = false;
//       } else {

//         magnetometer->getEvent(&mag_event);
//         gyroscope->getEvent(&gyro_event);
//         accelerometer->getEvent(&accel_event);
        
//         // 'Raw' values to match expectation of MotionCal
//         Serial.print("Raw:");
//         Serial.print(int(accel_event.acceleration.x*8192/9.8)); Serial.print(",");
//         Serial.print(int(accel_event.acceleration.y*8192/9.8)); Serial.print(",");
//         Serial.print(int(accel_event.acceleration.z*8192/9.8)); Serial.print(",");
//         Serial.print(int(gyro_event.gyro.x*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
//         Serial.print(int(gyro_event.gyro.y*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
//         Serial.print(int(gyro_event.gyro.z*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
//         Serial.print(int(mag_event.magnetic.x*10)); Serial.print(",");
//         Serial.print(int(mag_event.magnetic.y*10)); Serial.print(",");
//         Serial.print(int(mag_event.magnetic.z*10)); Serial.println("");

//         // unified data
//         Serial.print("Uni:");
//         Serial.print(accel_event.acceleration.x); Serial.print(",");
//         Serial.print(accel_event.acceleration.y); Serial.print(",");
//         Serial.print(accel_event.acceleration.z); Serial.print(",");
//         Serial.print(gyro_event.gyro.x, 4); Serial.print(",");
//         Serial.print(gyro_event.gyro.y, 4); Serial.print(",");
//         Serial.print(gyro_event.gyro.z, 4); Serial.print(",");
//         Serial.print(mag_event.magnetic.x); Serial.print(",");
//         Serial.print(mag_event.magnetic.y); Serial.print(",");
//         Serial.print(mag_event.magnetic.z); Serial.println("");
//         loopcount++;
//         receiveCalibration();

//         // occasionally print calibration
//         if (loopcount == 50 || loopcount > 100) {
//           Serial.print("Cal1:");
//           for (int i=0; i<3; i++) {
//             Serial.print(cal.accel_zerog[i], 3); 
//             Serial.print(",");
//           }
//           for (int i=0; i<3; i++) {
//             Serial.print(cal.gyro_zerorate[i], 3);
//             Serial.print(",");
//           }  
//           for (int i=0; i<3; i++) {
//             Serial.print(cal.mag_hardiron[i], 3); 
//             Serial.print(",");
//           }  
//           Serial.println(cal.mag_field, 3);
//           loopcount++;
//         }
//         if (loopcount >= 100) {
//           Serial.print("Cal2:");
//           for (int i=0; i<9; i++) {
//             Serial.print(cal.mag_softiron[i], 4); 
//             if (i < 8) Serial.print(',');
//           }
//           Serial.println();
//           loopcount = 0;
//         }
      
//         delay(10); 

//         caldata.compassCalibrated = true;
//         Serial.println("\nFully calibrated!");
//         Serial.println("--------------------------------");
//         Serial.println("Calibration Results: ");

//         Serial.println("--------------------------------");
//         caldata.compassCalibrated = true;
//         caldata.magOffsetX = calibrationData.mag_offset_x;
//         caldata.magOffsetY = calibrationData.mag_offset_y;
//         caldata.magOffsetZ = calibrationData.mag_offset_z;
//         caldata.gyroOffsetX = calibrationData.gyro_offset_x;
//         caldata.gyroOffsetY = calibrationData.gyro_offset_y;
//         caldata.gyroOffsetZ = calibrationData.gyro_offset_z;
//         caldata.accelOffsetX = calibrationData.accel_offset_x;
//         caldata.accelOffsetY = calibrationData.accel_offset_y;
//         caldata.accelOffsetZ = calibrationData.accel_offset_z;
//         caldata.accelRadius = calibrationData.accel_radius;
//         displaySensorOffsets(caldata);
//     }
//   }
// }

// void receiveCalibration() {
//   uint16_t crc;
//   byte b, i;

//   while (Serial.available()) {
//     b = Serial.read();
//     if (calcount == 0 && b != 117) {
//       // first byte must be 117
//       return;
//     }
//     if (calcount == 1 && b != 84) {
//       // second byte must be 84
//       calcount = 0;
//       return;
//     }
//     // store this byte
//     calibrationData[calcount++] = b;
//     if (calcount < 68) {
//       // full calibration message is 68 bytes
//       return;
//     }
//     // verify the crc16 check
//     crc = 0xFFFF;
//     for (i=0; i < 68; i++) {
//       crc = crc16_update(crc, calibrationData[i]);
//     }
//     if (crc == 0) {
//       // data looks good, use it
//       float offsets[16];
//       memcpy(offsets, calibrationData+2, 16*4);
//       cal.accel_zerog[0] = offsets[0];
//       cal.accel_zerog[1] = offsets[1];
//       cal.accel_zerog[2] = offsets[2];
      
//       cal.gyro_zerorate[0] = offsets[3];
//       cal.gyro_zerorate[1] = offsets[4];
//       cal.gyro_zerorate[2] = offsets[5];
      
//       cal.mag_hardiron[0] = offsets[6];
//       cal.mag_hardiron[1] = offsets[7];
//       cal.mag_hardiron[2] = offsets[8];

//       cal.mag_field = offsets[9];
      
//       cal.mag_softiron[0] = offsets[10];
//       cal.mag_softiron[1] = offsets[13];
//       cal.mag_softiron[2] = offsets[14];
//       cal.mag_softiron[3] = offsets[13];
//       cal.mag_softiron[4] = offsets[11];
//       cal.mag_softiron[5] = offsets[15];
//       cal.mag_softiron[6] = offsets[14];
//       cal.mag_softiron[7] = offsets[15];
//       cal.mag_softiron[8] = offsets[12];

//       if (! cal.saveCalibration()) {
//         Serial.println("**WARNING** Couldn't save calibration");
//       } else {
//         Serial.println("Wrote calibration");    
//       }
//       cal.printSavedCalibration();
//       calcount = 0;
//       return;
//     }
//     // look for the 117,84 in the data, before discarding
//     for (i=2; i < 67; i++) {
//       if (calibrationData[i] == 117 && calibrationData[i+1] == 84) {
//         // found possible start within data
//         calcount = 68 - i;
//         memmove(calibrationData, calibrationData + i, calcount);
//         return;
//       }
//     }
//     // look for 117 in last byte
//     if (calibrationData[67] == 117) {
//       calibrationData[0] = 117;
//       calcount = 1;
//     } else {
//       calcount = 0;
//     }
//   }
// }

// uint16_t crc16_update(uint16_t crc, uint8_t a)
// {
//   int i;
//   crc ^= a;
//   for (i = 0; i < 8; i++) {
//     if (crc & 1) {
//       crc = (crc >> 1) ^ 0xA001;
//     } else {
//       crc = (crc >> 1);
//     }
//   }
//   return crc;
// }

// float readCompass() {
// sox.getEvent(&acc, &gyr, &temp);
// lis3mdl.getEvent(&mag);
//   // put your main code here, to run repeatedly:

// thetaM=-atan2(acc.x()/9.8,acc.z()/9.8)/2/3.141592654*360;
// phiM=-atan2(acc.y()/9.8,acc.z()/9.8)/2/3.141592654*360;
// phiFnew=.95*phiFold+.05*phiM;
// thetaFnew=.95*thetaFold+.05*thetaM;
 
// dt=(millis()-millisOld)/1000.;
// millisOld=millis();
// theta=(theta+gyr.y()*dt)*.95+thetaM*.05;
// phi=(phi-gyr.x()*dt)*.95+ phiM*.05;
// thetaG=thetaG+gyr.y()*dt;
// phiG=phiG-gyr.x()*dt;
 
// phiRad=phi/360*(2*3.14);
// thetaRad=theta/360*(2*3.14);
 
// Xm=mag.x()*cos(thetaRad)-mag.y()*sin(phiRad)*sin(thetaRad)+mag.z()*cos(phiRad)*sin(thetaRad);
// Ym=mag.y()*cos(phiRad)+mag.z()*sin(phiRad);
 
// psi=atan2(Ym,Xm)/(2*3.14)*360;
// phiFold=phiFnew;
// thetaFold=thetaFnew;

// delay(BNO055_SAMPLERATE_DELAY_MS);
//   if(!caldata.compassCalibrated){
//     psi = 999.9;
//   }

//   //Serial.println(psi);
//   //psi = 0.0;
//   // psi = psi + 180;
//   if(caldata.compassOffset >= 0){
//     if(psi < caldata.compassOffset){
//       psi = psi - caldata.compassOffset + 360.0;
//     }else{
//       psi = psi - caldata.compassOffset;
//     }
//   }else{
//       psi = psi + (-1.0 * caldata.compassOffset);
//       if(psi >= 360)
//       {
//           psi = psi - 360;
//       }
//   }

// return psi;
// }