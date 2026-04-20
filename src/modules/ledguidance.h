#ifndef LEDGUIDANCE_H
#define LEDGUIDANCE_H

#include <Arduino.h>

void LedGuidanceHeading(float relativeHeading, bool bEnable, bool debug, int distancePOI);
void LedGuidanceStop();

#endif // LEDGUIDANCE_H
