#include <Arduino.h>
#ifndef SELECTEDMAP_H
#define SELECTEDMAP_H

struct SelectedMap {
  // id, name, country, area, mapfile, kmlfile
  int map_id; // 1
  char map_name[32] = ""; // "Test area"
  char map_area[32] = ""; // "Kaag en Braassem"
  char map_country[64] = ""; // "Netherlands"
  char map_pngFile[32] = ""; // "kaagenbraassem.png"
  int imageWidth = 0;
  int imageHeight = 0;
  char map_kmlFile[16] = ""; // "kaagenbraassem.kml"
  double realWorldHeight = 0.0;
  double realWorldWidth = 0.0;
  float scaleHeight = 1.0;
  float scaleWidth = 1.0;
  double north = 0.0;
  double west = 0.0;
  double south = 0.0;
  double east = 0.0;
  float rotation = 0.0;
  int radius = 63713000;
};

#endif