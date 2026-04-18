#ifndef HAPTICFEEDBACK_H
#define HAPTICFEEDBACK_H

#include <Arduino.h>
#include "../structs/caldata.h"
#include "../structs/config.h"
#include "../structs/debugsettings.h"

void HapticFeedbackHeading(float relativeHeading, bool bEnable, bool debug, int distancePOI);
void HapticFeedbackStop();
void HapticFeedbackTick();  // Call every loop() iteration — drives the non-blocking pulse state machine.

#endif // HAPTICFEEDBACK_H