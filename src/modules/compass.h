#ifndef COMPASS_H
#define COMPASS_H

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include "SensorFusion.h"

void magnometerSetup();
double getCompassHeading();

#endif