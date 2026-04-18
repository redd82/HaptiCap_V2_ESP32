// File: sensordata.h
#ifndef SENSORDATA_H
#define SENSORDATA_H

struct SensorData {
  char sensorName[8] = "";
  int gpsTime = 0;
  double ownLat = 0.0;
  double ownLon= 0.0;
  float compassHeading = 0.0;
  char compassCardinal[8] = "";
  double homeBaseLat= 0.0;
  double homeBaseLon= 0.0;
  float homeBaseBearing= 0.0;
  char homeBaseCardinal[8] = "";
  int homeBaseDistance= 0;
  float relheading = 0.0;
  float relheadingHomeBase = 0.0;
  double wayPointLat = 0.0;
  double wayPointLon = 0.0;
  float wayPointBearing= 0.0;
  char wayPointCardinal[8] = "";
  int wayPointDistance = 0;
  int nrOfSatellites = 0;
};

#endif