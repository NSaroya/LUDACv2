#ifndef LUDAC_GPS_H
#define LUDAC_GPS_H

#include <Adafruit_GPS.h>
#include <Adafruit_PMTK.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
#include "Math.h"
#include <NMEA_data.h>

// Definitions for GPS module
#define GPSECHO false
#define GPSRST 3
#define EARTH_RADIUS 6371.0  // km

// Delay in milliseconds to wait for GPS initialization
#define INIT_DELAY 1000

// Time difference threshold for Haversine calculation in milliseconds
#define TEN_SECONDS 10000

// Structure to hold latitude, longitude, and fix status
// struct LatLong {
//   float latitude;       // Latitude in decimal degrees
//   float longitude;      // Longitude in decimal degrees
//   bool hasValidFix;     // Flag indicating if GPS has a valid fix
//   uint32_t timestamp;   // Timestamp of GPS fix in milliseconds
// };

// struct LatLong {
//   float latitude;       // Latitude in decimal degrees
//   float longitude;      // Longitude in decimal degrees
//   float altitude;       // Altitude in meters
//   int gpsQuality;       // GPS quality indicator
//   bool hasValidFix;     // Flag indicating if GPS has a valid fix
//   String utcTime;       // UTC time of position fix
//   uint32_t timestamp;   // Timestamp of GPS fix in milliseconds
// };

struct LatLong {
  float latitude;       // Latitude in decimal degrees
  float longitude;      // Longitude in decimal degrees
  float altitude;       // Altitude in meters
  int gpsQuality;       // GPS quality indicator
  bool hasValidFix;     // Flag indicating if GPS has a valid fix
  String utcTime;       // UTC time of position fix
  uint32_t timestamp;   // Timestamp of GPS fix in milliseconds

  // Function to reset all fields to null or zeros
  void reset() {
    latitude = 0.0;
    longitude = 0.0;
    altitude = 0.0;
    gpsQuality = 0;
    hasValidFix = false;
    utcTime = "";
    timestamp = 0;
  }
};

// Structure to hold Haversine distance and time difference
struct Haversine {
  float distance;   // Haversine distance in kilometers
  int timeDiff;     // Time difference in seconds
};

// GPS initialization and GPS fix related functions
void initLudacGPS();  // Initialize GPS module
String getGPSdata();
bool receivedGPSfix();  // Check if a new GPS fix has been received
bool hasNewGPSFix(struct LatLong *, struct LatLong *);  // Check if a new GPS fix is available
bool getGPSfix();  // Check if GPS has a fix
uint8_t getGPSfixquality();  // Get GPS fix quality

// Get various types of GPS information
void getGPStime(String &);  // Get GPS time in HH:MM:SS format
void getGPSdate(String &);  // Get GPS date in YYYY-MM-DD format
void getLatLong(struct LatLong *);  // Get latitude and longitude from GPS fix
float getGPSspeed();  // Get GPS speed in meters per second
float getGPSangle();  // Get GPS angle in degrees
float getGPSaltitude();  // Get GPS altitude in meters
uint8_t getGPSsatellites();  // Get number of satellites used in GPS fix
float getGPStimeSinceLastFix();  // Get time since last GPS fix in seconds
float getGPSlastTime();  // Get time since last GPS time update in seconds
float getGPSlastDate();  // Get time since last GPS date update in seconds
void getHaversineDistance(struct LatLong *, struct LatLong *, struct Haversine *);  // Calculate Haversine distance between two points

// Convert LatLong to various formats
void convertLatLongForDisplay(struct LatLong *, String &);  // Convert latitude and longitude for display
void convertLatLongToString(struct LatLong *, String &);  // Convert latitude and longitude to string format
void convertStringToLatLong(String, struct LatLong *);  // Convert string format latitude and longitude to LatLong structure
float convertDMtoDecimalDegrees(float value);  // Convert Degree-Minutes to Decimal Degrees
bool isOKtoCalculateHaversine(struct LatLong *, struct LatLong *);  // Check if it is okay to calculate Haversine distance
int calculateTimeDiff(uint32_t);  // Calculate time difference since given timestamp

#endif  // LUDAC_GPS_H
