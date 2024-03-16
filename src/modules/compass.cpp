#include "compass.h"

// global objects and variables
extern SF fusion;
extern Adafruit_LIS3MDL lis3mdl;
extern Adafruit_LSM6DS3TRC lsm6ds3strc;
extern String jsonDir;
extern String fileCalDataJSON;
extern CalData caldata;
extern double DEG_2_RAD;
extern float deltat;
extern bool magnetometerCalibrated;
extern bool compassCalibrated;
extern float compassheading;

// local objects and variables
Adafruit_Sensor_Calibration_EEPROM cal; 
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
sensors_event_t mag_event, gyro_event, accel_event, temp_event;

struct SensorDataMag
{
  float min_x, max_x, mid_x, min_y, max_y, mid_y, min_z, max_z, mid_z;
};

struct SensorDataGyro
{
  float min_x, max_x, mid_x, min_y, max_y, mid_y, min_z, max_z, mid_z;
};

struct SensorDataAccel
{
  float min_x, max_x, mid_x, min_y, max_y, mid_y, min_z, max_z, mid_z;
};

SensorDataMag magData;
SensorDataGyro gyroData;
SensorDataAccel accelData;

int loopcount = 0;

byte calDataBuffer[68]; // buffer to receive magnetic calibration data
byte calcount=0;

int calSamples = 1000;
float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float pitch, roll, yaw;
float deltat;

float thetaM;
float phiM;
float thetaFold=0;
float thetaFnew;
float phiFold=0;
float phiFnew;
float thetaG=0;
float phiG=0;
float theta;
float phi;
float thetaRad;
float phiRad;
 
float Xm;
float Ym;
float psi;

float dt;
unsigned long millisOld;

void setupLis3md(){
    if (! lis3mdl.begin_I2C()) { 
      Serial.println("Failed to find LIS3MDL chip");
    while (1) { delay(10); }
  }
  Serial.println("LIS3MDL Found!");

  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  lis3mdl.getEvent(&mag_event);
  magData.min_x = magData.max_x = mag_event.magnetic.x;
  magData.min_y = magData.max_y = mag_event.magnetic.y;
  magData.min_z = magData.max_z = mag_event.magnetic.z;
}

void setupLsm6ds3strc(){
  if (! lsm6ds3strc.begin_I2C()) {
    Serial.println("Failed to find LSM6DS3TRC chip");
    while (1) { delay(10); }
  }
  Serial.println("LSM6DS3TRC Found!");
  lsm6ds3strc.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds3strc.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lsm6ds3strc.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds3strc.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds3strc.configInt1(false, false, true); // accelerometer DRDY on INT1
  lsm6ds3strc.configInt2(false, true, false); // gyro DRDY on INT2
}

void debugMag(){
  lis3mdl.getEvent(&mag_event);
  Serial.print("X: "); Serial.print(mag_event.magnetic.x); Serial.print(" uT\t");
  Serial.print("Y: "); Serial.print(mag_event.magnetic.y); Serial.print(" uT\t");
  Serial.print("Z: "); Serial.print(mag_event.magnetic.z); Serial.print(" uT\t");
  Serial.println("Temp: " + String(mag_event.temperature) + "C");
  delay(100);
}

void debugGyroAccel(){
  lsm6ds3strc.getEvent(&accel_event, &gyro_event, &temp_event);
  Serial.print("Accel X: "); Serial.print(accel_event.acceleration.x); Serial.print(" m/s^2\t");
  Serial.print("Y: "); Serial.print(accel_event.acceleration.y); Serial.print(" m/s^2\t");
  Serial.print("Z: "); Serial.print(accel_event.acceleration.z); Serial.print(" m/s^2\t");
  Serial.print("Gyro X: "); Serial.print(gyro_event.gyro.x); Serial.print(" rad/s\t");
  Serial.print("Y: "); Serial.print(gyro_event.gyro.y); Serial.print(" rad/s\t");
  Serial.print("Z: "); Serial.print(gyro_event.gyro.z); Serial.print(" rad/s\t");
  Serial.println("Temp: " + String(temp_event.temperature) + "C");
  delay(100);
}

void plotterDataGyroAccelMag(){
  lsm6ds3strc.getEvent(&accel_event, &gyro_event, &temp_event);
  lis3mdl.getEvent(&mag_event);
  Serial.print(accel_event.acceleration.x); Serial.print(",");
  Serial.print(accel_event.acceleration.y); Serial.print(",");
  Serial.print(accel_event.acceleration.z); Serial.print(",");
  Serial.print(gyro_event.gyro.x); Serial.print(",");
  Serial.print(gyro_event.gyro.y); Serial.print(",");
  Serial.print(gyro_event.gyro.z); Serial.print(",");
  Serial.print(mag_event.magnetic.x); Serial.print(",");
  Serial.print(mag_event.magnetic.y); Serial.print(",");
  Serial.print(mag_event.magnetic.z); Serial.print(",");
  //Serial.println(temp_event.temperature);
  delay(100);
}

void calibrateMagHardIron(){
  Serial.println(F("Move the sensor in a figure 8 until done!"));
  Serial.print(F("Fetching samples in 3..."));
  delay(1000);
  Serial.print("2...");
  delay(1000);
  Serial.print("1...");
  delay(1000);
  Serial.println("NOW!");
  int i = 0;
  while(i < calSamples){
    lis3mdl.getEvent(&mag_event);
    float x = mag_event.magnetic.x;
    float y = mag_event.magnetic.y;
    float z = mag_event.magnetic.z;

    Serial.print("Mag: (");
    Serial.print(x); Serial.print(", ");
    Serial.print(y); Serial.print(", ");
    Serial.print(z); Serial.print(")");

    magData.min_x = min(magData.min_x, x);
    magData.min_y = min(magData.min_y, y);
    magData.min_z = min(magData.min_z, z);

    magData.max_x = max(magData.max_x, x);
    magData.max_y = max(magData.max_y, y);
    magData.max_z = max(magData.max_z, z);

    magData.mid_x = (magData.max_x + magData.min_x) / 2;
    magData.mid_y = (magData.max_y + magData.min_y) / 2;
    magData.mid_z = (magData.max_z + magData.min_z) / 2;
    Serial.print(" Hard offset: (");
    Serial.print(magData.mid_x); Serial.print(", ");
    Serial.print(magData.mid_y); Serial.print(", ");
    Serial.print(magData.mid_z); Serial.print(")");  

    Serial.print(" Field: (");
    Serial.print((magData.max_x - magData.min_x)/2); Serial.print(", ");
    Serial.print((magData.max_y - magData.min_y)/2); Serial.print(", ");
    Serial.print((magData.max_z - magData.min_z)/2); Serial.println(")");
    i++;    
    delay(10); 
  }
  caldata.magOffsetX = magData.mid_x;
  caldata.magOffsetY = magData.mid_y;
  caldata.magOffsetZ = magData.mid_z;
  Serial.println(F("Magneto hard iron calibration done!"));
    delay(3000);
}

void calibrateGyroHardIron(){
  Serial.println(F("Place gyro on flat, stable surface!"));
  Serial.print(F("Fetching samples in 3..."));
  delay(1000);
  Serial.print("2...");
  delay(1000);
  Serial.print("1...");
  delay(1000);
  Serial.println("NOW!");
  float x, y, z;
  for (uint16_t sample = 0; sample < calSamples; sample++) {
    lsm6ds3strc.getEvent(&accel_event, &gyro_event, &temp_event);
    x = gyro_event.gyro.x;
    y = gyro_event.gyro.y;
    z = gyro_event.gyro.z;
    Serial.print(F("Gyro: ("));
    Serial.print(x); Serial.print(", ");
    Serial.print(y); Serial.print(", ");
    Serial.print(z); Serial.print(")");

    gyroData.min_x = min(gyroData.min_x, x);
    gyroData.min_y = min(gyroData.min_y, y);
    gyroData.min_z = min(gyroData.min_z, z);
  
    gyroData.max_x = max(gyroData.max_x, x);
    gyroData.max_y = max(gyroData.max_y, y);
    gyroData.max_z = max(gyroData.max_z, z);
  
    gyroData.mid_x = (gyroData.max_x + gyroData.min_x) / 2;
    gyroData.mid_y = (gyroData.max_y + gyroData.min_y) / 2;
    gyroData.mid_z = (gyroData.max_z + gyroData.min_z) / 2;

    Serial.print(F(" Zero rate offset: ("));
    Serial.print(gyroData.mid_x, 4); Serial.print(", ");
    Serial.print(gyroData.mid_y, 4); Serial.print(", ");
    Serial.print(gyroData.mid_z, 4); Serial.print(")");  
  
    Serial.print(F(" rad/s noise: ("));
    Serial.print(gyroData.max_x - gyroData.min_x, 3); Serial.print(", ");
    Serial.print(gyroData.max_y - gyroData.min_y, 3); Serial.print(", ");
    Serial.print(gyroData.max_z - gyroData.min_z, 3); Serial.println(")");   
    delay(10);
  }
  Serial.println(F("\n\nFinal zero rate offset in radians/s: "));
  Serial.print(gyroData.mid_x, 4); Serial.print(", ");
  Serial.print(gyroData.mid_y, 4); Serial.print(", ");
  Serial.println(gyroData.mid_z, 4);
  caldata.gyroOffsetX = gyroData.mid_x;
  caldata.gyroOffsetY = gyroData.mid_y;
  caldata.gyroOffsetZ = gyroData.mid_z;
  Serial.println(F("Gyro hard iron calibration done!"));
  delay(3000);
}

void motionCal(){
  while(1){
  lsm6ds3strc.getEvent(&accel_event, &gyro_event, &temp_event);
  lis3mdl.getEvent(&mag_event);
  Serial.print("Raw:");
  Serial.print(int(accel_event.acceleration.x*8192/9.8)); Serial.print(",");
  Serial.print(int(accel_event.acceleration.y*8192/9.8)); Serial.print(",");
  Serial.print(int(accel_event.acceleration.z*8192/9.8)); Serial.print(",");
  Serial.print(int(gyro_event.gyro.x*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
  Serial.print(int(gyro_event.gyro.y*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
  Serial.print(int(gyro_event.gyro.z*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
  Serial.print(int(mag_event.magnetic.x*10)); Serial.print(",");
  Serial.print(int(mag_event.magnetic.y*10)); Serial.print(",");
  Serial.print(int(mag_event.magnetic.z*10)); Serial.println("");
  loopcount++;
  receiveCalibration();

  if (loopcount == 50 || loopcount > 100) {
    Serial.print("Cal1:");
    Serial.print(caldata.accelOffsetX, 3); 
    Serial.print(",");
    Serial.print(caldata.accelOffsetY, 3); 
    Serial.print(",");
    Serial.print(caldata.accelOffsetZ, 3); 
    Serial.print(",");

    Serial.print(caldata.gyroOffsetX, 3);
    Serial.print(",");
    Serial.print(caldata.gyroOffsetY, 3);
    Serial.print(",");
    Serial.print(caldata.gyroOffsetZ, 3);      
    Serial.print(",");
  
    Serial.print(caldata.magOffsetX, 3); 
    Serial.print(",");
    Serial.print(caldata.magOffsetY, 3); 
    Serial.print(",");
    Serial.print(caldata.magOffsetZ, 3); 
    Serial.print(",");            

    Serial.println(caldata.magField, 3);
    loopcount++;
  }
  if (loopcount >= 100) {
    Serial.print("Cal2:");
    for (int i=0; i<9; i++) {
      Serial.print(cal.mag_softiron[i], 4); 
      if (i < 8) Serial.print(',');
    }
    Serial.println();
    loopcount = 0;
  }

  delay(10); 
  }
}

void receiveCalibration() {
  uint16_t crc;
  byte b, i;

  while (Serial.available()) {
    b = Serial.read();
    if (calcount == 0 && b != 117) {
      // first byte must be 117
      return;
    }
    if (calcount == 1 && b != 84) {
      // second byte must be 84
      calcount = 0;
      return;
    }
    // store this byte
    calDataBuffer[calcount++] = b;
    if (calcount < 68) {
      // full calibration message is 68 bytes
      return;
    }
    // verify the crc16 check
    crc = 0xFFFF;
    for (i=0; i < 68; i++) {
      crc = crc16_update(crc, calDataBuffer[i]);
    }
    if (crc == 0) {
      // data looks good, use it
      float offsets[16];
      memcpy(offsets, calDataBuffer+2, 16*4);
      cal.accel_zerog[0] = offsets[0];
      cal.accel_zerog[1] = offsets[1];
      cal.accel_zerog[2] = offsets[2];
      
      cal.gyro_zerorate[0] = offsets[3];
      cal.gyro_zerorate[1] = offsets[4];
      cal.gyro_zerorate[2] = offsets[5];
      
      cal.mag_hardiron[0] = offsets[6];
      cal.mag_hardiron[1] = offsets[7];
      cal.mag_hardiron[2] = offsets[8];

      cal.mag_field = offsets[9];
      
      cal.mag_softiron[0] = offsets[10];
      cal.mag_softiron[1] = offsets[13];
      cal.mag_softiron[2] = offsets[14];
      cal.mag_softiron[3] = offsets[13];
      cal.mag_softiron[4] = offsets[11];
      cal.mag_softiron[5] = offsets[15];
      cal.mag_softiron[6] = offsets[14];
      cal.mag_softiron[7] = offsets[15];
      cal.mag_softiron[8] = offsets[12];

      if (! cal.saveCalibration()) {
        Serial.println("**WARNING** Couldn't save calibration");
      } else {
        Serial.println("Wrote calibration");    
      }
      cal.printSavedCalibration();
      calcount = 0;
      return;
    }
    // look for the 117,84 in the data, before discarding
    for (i=2; i < 67; i++) {
      if (calDataBuffer[i] == 117 && calDataBuffer[i+1] == 84) {
        // found possible start within data
        calcount = 68 - i;
        memmove(calDataBuffer, calDataBuffer + i, calcount);
        return;
      }
    }
    // look for 117 in last byte
    if (calDataBuffer[67] == 117) {
      calDataBuffer[0] = 117;
      calcount = 1;
    } else {
      calcount = 0;
    }
  }
}

uint16_t crc16_update(uint16_t crc, uint8_t a)
{
  int i;
  crc ^= a;
  for (i = 0; i < 8; i++) {
    if (crc & 1) {
      crc = (crc >> 1) ^ 0xA001;
    } else {
      crc = (crc >> 1);
    }
  }
  return crc;
}

void setup_sensors(void) { //lis3md lsm6ds3strc
  setupLis3md();
  setupLsm6ds3strc();
  calibrateCompass();
}

void compassSetup(){
  setup_sensors();
}

bool calibrateCompass(){
  if(!caldata.compassCalibrated){
    calibrateMagHardIron();
    calibrateGyroHardIron();
    caldata.compassCalibrated = true;
    saveCalibrationDataToJSON();
    saveCalibrationData(LittleFS, (jsonDir + fileCalDataJSON).c_str(), caldata);
  }
  return false;
}

void readMag(){

}

void readGyro(){

}

void readAccel(){

}

float readCompass(){
  return 0.0;
}
