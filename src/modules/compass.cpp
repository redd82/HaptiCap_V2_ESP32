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

// ── Low-pass filter for heading output ───────────────────────────────────────
// Alpha controls smoothing: lower = smoother but slower response.
// At ~100 ms call period, 0.12 gives roughly a 700 ms time constant.
static const float HEADING_ALPHA = 0.12f;
static float filteredHeading = -1.0f;   // negative = uninitialized sentinel

// Wraparound-safe exponential moving average for circular angles.
static float headingLowPass(float newVal) {
    if (filteredHeading < 0.0f) {
        filteredHeading = newVal;
        return filteredHeading;
    }
    float delta = newVal - filteredHeading;
    if      (delta >  180.0f) delta -= 360.0f;
    else if (delta < -180.0f) delta += 360.0f;
    filteredHeading += HEADING_ALPHA * delta;
    if      (filteredHeading <   0.0f) filteredHeading += 360.0f;
    else if (filteredHeading >= 360.0f) filteredHeading -= 360.0f;
    return filteredHeading;
}
// ─────────────────────────────────────────────────────────────────────────────

static void updateMagEvent() {
  // Some LIS3MDL board variants never raise magneticFieldAvailable() reliably
  // over I2C, so use direct event reads for robust sampling.
  lis3mdl.getEvent(&mag_event);
}

static bool readFreshMagSample(float &x, float &y, float &z) {
  // Retry briefly so calibration reads are less likely to reuse stale values.
  const float prevX = mag_event.magnetic.x;
  const float prevY = mag_event.magnetic.y;
  const float prevZ = mag_event.magnetic.z;

  for (int tries = 0; tries < 10; ++tries) {
    updateMagEvent();
    x = mag_event.magnetic.x;
    y = mag_event.magnetic.y;
    z = mag_event.magnetic.z;
    float delta = fabsf(x - prevX) + fabsf(y - prevY) + fabsf(z - prevZ);
    if (delta >= 0.001f) {
      return true;
    }
    delay(2);
  }

  // Return the latest sample even if it appears unchanged.
  x = mag_event.magnetic.x;
  y = mag_event.magnetic.y;
  z = mag_event.magnetic.z;
  return false;
}

void setupLis3md(){
    if (! lis3mdl.begin_I2C()) { 
      Serial.println("Failed to find LIS3MDL chip");
    while (1) { delay(10); }
  }
  Serial.println("LIS3MDL Found!");

  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  // 155 Hz + high performance mode is a stable combo over I2C for live calibration.
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  updateMagEvent();
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
  updateMagEvent();
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
  updateMagEvent();
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
  // Re-init magnetometer right before calibration to ensure it is in continuous mode.
  setupLis3md();
  Serial.println(F("Move the sensor in a figure 8 until done!"));
  Serial.print(F("Fetching samples in 3..."));
  delay(1000);
  Serial.print("2...");
  delay(1000);
  Serial.print("1...");
  delay(1000);
  Serial.println("NOW!");

  // Reset extrema for a fresh calibration run.
  float seedX = 0.0f;
  float seedY = 0.0f;
  float seedZ = 0.0f;
  readFreshMagSample(seedX, seedY, seedZ);
  magData.min_x = magData.max_x = seedX;
  magData.min_y = magData.max_y = seedY;
  magData.min_z = magData.max_z = seedZ;

  float prevX = seedX;
  float prevY = seedY;
  float prevZ = seedZ;
  int stagnantSamples = 0;
  bool stagnantWarned = false;

  int i = 0;
  while(i < calSamples){
    delay(20);
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    bool fresh = readFreshMagSample(x, y, z);

    float delta = fabsf(x - prevX) + fabsf(y - prevY) + fabsf(z - prevZ);
    if (delta < 0.01f) {
      stagnantSamples++;
    } else {
      stagnantSamples = 0;
      stagnantWarned = false;
    }
    if (stagnantSamples > 50 && !stagnantWarned) {
      Serial.println(F("WARNING: Magnetometer samples appear static. Check wiring/power and rotate on all axes."));
      stagnantWarned = true;
    }
    prevX = x;
    prevY = y;
    prevZ = z;

    Serial.print("Mag raw[");
    Serial.print(i);
    Serial.print("]: (");
    Serial.print(x, 4); Serial.print(", ");
    Serial.print(y, 4); Serial.print(", ");
    Serial.print(z, 4); Serial.print(")");
    if (!fresh) {
      Serial.print(" [stale]");
    }

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
    Serial.print(magData.mid_x, 4); Serial.print(", ");
    Serial.print(magData.mid_y, 4); Serial.print(", ");
    Serial.print(magData.mid_z, 4); Serial.print(")");  

    Serial.print(" Field: (");
    Serial.print((magData.max_x - magData.min_x)/2, 4); Serial.print(", ");
    Serial.print((magData.max_y - magData.min_y)/2, 4); Serial.print(", ");
    Serial.print((magData.max_z - magData.min_z)/2, 4); Serial.println(")");
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

  // Seed extrema from the first reading to avoid uninitialized min/max state.
  lsm6ds3strc.getEvent(&accel_event, &gyro_event, &temp_event);
  gyroData.min_x = gyroData.max_x = gyro_event.gyro.x;
  gyroData.min_y = gyroData.max_y = gyro_event.gyro.y;
  gyroData.min_z = gyroData.max_z = gyro_event.gyro.z;

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

void setup_sensors(void) { //lis3md lsm6ds3strc
  setupLis3md();
  setupLsm6ds3strc();
  calibrateCompass();
}

void compassSetup(){
  millisOld = millis();
  filteredHeading = -1.0f;
  setup_sensors();
}

bool calibrateCompass(){
  if(!caldata.compassCalibrated){
    calibrateMagHardIron();
    calibrateGyroHardIron();
    caldata.compassCalibrated = true;
    saveCalibrationDataToJSON();
    saveCalibrationData(LittleFS, (jsonDir + fileCalDataJSON).c_str(), caldata);
  } else {
    Serial.println(F("Compass calibration skipped (compassCalibrated=true)"));
  }
  return false;
}

void readMag(){
  updateMagEvent();
}

void readGyro(){
  lsm6ds3strc.getEvent(&accel_event, &gyro_event, &temp_event);
}

void readAccel(){
  // accel_event is populated together with gyro in readGyro()
}

static float normalize360(float angleDeg) {
  while (angleDeg < 0.0f) angleDeg += 360.0f;
  while (angleDeg >= 360.0f) angleDeg -= 360.0f;
  return angleDeg;
}

float setCompassNorth() {
    // Read current heading with existing calibration and use it to rotate
    // the offset so the present direction becomes 0° (North).
    float currentHeading = readCompass();
    caldata.compassOffset = normalize360(caldata.compassOffset - currentHeading);
    filteredHeading = -1.0f;
    compassheading = readCompass();

    Serial.print(F("Set North applied. New compassOffset="));
    Serial.println(caldata.compassOffset, 4);
    Serial.print(F("New heading="));
    Serial.println(compassheading, 2);
    return caldata.compassOffset;
}

float readCompass() {
    // ── 1. Read raw sensor data ───────────────────────────────────────────────
    lsm6ds3strc.getEvent(&accel_event, &gyro_event, &temp_event);
    updateMagEvent();

    // ── 2. Compute timestep (clamp to sane range) ────────────────────────────
    unsigned long now = millis();
    dt = (now - millisOld) * 0.001f;
    millisOld = now;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.01f;

    // ── 3. Gyro: subtract zero-rate bias (rad/s) ─────────────────────────────
    gx = gyro_event.gyro.x - caldata.gyroOffsetX;
    gy = gyro_event.gyro.y - caldata.gyroOffsetY;
    gz = gyro_event.gyro.z - caldata.gyroOffsetZ;

    // ── 4. Accelerometer: subtract bias (m/s²) ───────────────────────────────
    // Guard: accel offsets > 5 m/s² are physically impossible and indicate
    // stale data from an old BNO055 calibration file (different units).
    // Fall back to zero offset to keep the Mahony filter sane.
    float safeAOX = (fabsf(caldata.accelOffsetX) < 5.0f) ? caldata.accelOffsetX : 0.0f;
    float safeAOY = (fabsf(caldata.accelOffsetY) < 5.0f) ? caldata.accelOffsetY : 0.0f;
    float safeAOZ = (fabsf(caldata.accelOffsetZ) < 5.0f) ? caldata.accelOffsetZ : 0.0f;
    ax = accel_event.acceleration.x - safeAOX;
    ay = accel_event.acceleration.y - safeAOY;
    az = accel_event.acceleration.z - safeAOZ;

    // ── 5. Magnetometer: hard-iron then soft-iron correction ─────────────────
    float mx_raw = mag_event.magnetic.x - caldata.magOffsetX;
    float my_raw = mag_event.magnetic.y - caldata.magOffsetY;
    float mz_raw = mag_event.magnetic.z - caldata.magOffsetZ;
    // magSoftIron is a row-major 3x3 matrix (identity by default = no effect)
    mx = caldata.magSoftIron[0]*mx_raw + caldata.magSoftIron[1]*my_raw + caldata.magSoftIron[2]*mz_raw;
    my = caldata.magSoftIron[3]*mx_raw + caldata.magSoftIron[4]*my_raw + caldata.magSoftIron[5]*mz_raw;
    mz = caldata.magSoftIron[6]*mx_raw + caldata.magSoftIron[7]*my_raw + caldata.magSoftIron[8]*mz_raw;

    // ── 6. Mahony AHRS: fuse gyro + accel + mag ──────────────────────────────
    // Produces a stable quaternion-tracked orientation referenced to mag north.
    fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

    // ── 7. Tilt-compensated heading from fused pitch/roll + corrected mag ─────
    // Using the geometric formulation keeps heading independent of the filter's
    // yaw initialisation; pitch/roll stabilise after ~1-2 seconds.
    float pitchRad = fusion.getPitch() * (float)DEG_2_RAD;
    float rollRad  = fusion.getRoll()  * (float)DEG_2_RAD;

    float cosPitch = cosf(pitchRad);
    float sinPitch = sinf(pitchRad);
    float cosRoll  = cosf(rollRad);
    float sinRoll  = sinf(rollRad);

    // Project the corrected mag vector into the horizontal plane.
    Xm =  mx * cosPitch + mz * sinPitch;
    Ym =  mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch;

    // Use right-handed heading sign so clockwise rotation increases degrees.
    psi = atan2f(Ym, Xm) * (180.0f / M_PI);

    // ── 8. Apply declination offset and normalise to [0, 360) ────────────────
    psi += caldata.compassOffset;
    if      (psi <   0.0f) psi += 360.0f;
    else if (psi >= 360.0f) psi -= 360.0f;

    // ── 9. Smooth output to suppress high-frequency jitter ───────────────────
    compassheading = headingLowPass(psi);
    return compassheading;
}
