#ifndef WAYPOINTSMAP_H
#define WAYPOINTSMAP_H

#define NROFWAYPOINTS 30

struct WaypointsMap {
  double homeBase[2] = { 0.0 };
  double wayPoint [NROFWAYPOINTS][2];
};

#endif