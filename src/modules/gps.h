#ifndef GPS_H
#define GPS_H

#include <TinyGPSPlus.h>
#include "../structs/config.h"
#include "../structs/sensordata.h"

String getGPSTimeMinsSecs();
String getGPSTimeMins();
String getGPSDate();
unsigned long distance2waypoint(float waypoint_latt, float waypoint_long);
float coarse2waypoint(float waypoint_latt, float waypoint_long);
float CalcRelHeading(float compforheading,float coarseforWaypoint);
// void checkGPSFix();

#endif