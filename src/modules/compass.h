#ifndef COMPASS_H
#define COMPASS_H

#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS3TRC.h>
#include "Adafruit_Sensor_Calibration.h"
#include <Adafruit_Sensor.h>
#include "SensorFusion.h"
#include "filehandler.h"
#include "../structs/caldata.h"

void compassSetup();
void displaySensorDetails(void);
void displaySensorStatus(void);
void displayCalStatus(void);
void displaySensorOffsets(const CalData &caldata);
void debugMag();
void debugGyroAccel();
void plotterDataGyroAccelMag();
bool calibrateCompass();
float readCompass();
float setCompassNorth();
#endif