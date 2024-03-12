#ifndef COMPASS_H
#define COMPASS_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "SensorFusion.h"
#include "../structs/caldata.h"

void compassSetup();
void displaySensorDetails(void);
void displaySensorStatus(void);
void displayCalStatus(void);
void displaySensorOffsets(const CalData &caldata);
void setupBNO055();
double getCompassHeading();
float readCompass();
void calibrateCompass();

#endif