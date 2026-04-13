#ifndef HAPTICFEEDBACK_H
#define HAPTICFEEDBACK_H

#include <Arduino.h>
#include "../structs/caldata.h"
#include "../structs/config.h"
#include "../structs/debugsettings.h"

void HapticFeedbackHeading(float relativeHeading,bool bEnable,bool debug,int distancePOI,CalData caldata,Config config);


#endif // HAPTICFEEDBACK_H