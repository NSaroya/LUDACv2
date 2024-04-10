/**
 * @file LUDAC_GPS.cpp
 * @brief GPS Parsing Module.
 * 
 * This module provides functionality to parse the GPS data
 * into various formats
 */

#include "LUDAC_GPS.h"

// Define verbose printing macro for debugging
#define VERBOSE_PRINT_GPS(x) \
    Serial.print(millis()); \
    Serial.print(": "); \
    Serial.println(x);

// Global variable for GPS module
SoftwareSerial GPSSerial(26,27);
Adafruit_GPS GPS(&GPSSerial);

// Timezone in hours (GMT -6)
const uint8_t timezone = -6;

/**
 * @brief Initialize GPS module.
 */
void initLudacGPS() {
  // Begin communication with GPS module
  GPSSerial.begin(9600);
  // GPS.begin(9600);

  // Set NMEA output and update rate
  GPSSerial.print("PMTK_SET_NMEA_OUTPUT_GGAONLY"); //PMTK_SET_NMEA_OUTPUT_GGAONLY PMTK_SET_NMEA_OUTPUT_RMCGGA
  GPSSerial.print("PMTK_SET_NMEA_UPDATE_1HZ");

  // Enable antenna
  GPSSerial.print("PGCMD_ANTENNA");

  // Wait for GPS to initialize
  delay(INIT_DELAY);

  // Set GPS reset pin as output and keep it HIGH
  pinMode(GPSRST, OUTPUT);
  digitalWrite(GPSRST, HIGH);
}

/**
 * @brief Check if a new GPS fix has been received.
 * 
 * @return True if a new fix is received, false otherwise.
 */
bool receivedGPSfix() {
  char c = GPS.read();
  VERBOSE_PRINT_GPS("Received character from GPS Device: " + String(c));

  if (GPS.newNMEAreceived()) {
    VERBOSE_PRINT_GPS("A new NMEA line has been received");
    if (!GPS.parse(GPS.lastNMEA())) {
      return false;
    }
  }

  // Check if GPS has a fix
  if (!GPS.fix) {
    VERBOSE_PRINT_GPS("GPS does not have a fix yet");
    return false;
  }

  VERBOSE_PRINT_GPS("GPS has a fix");
  return true;
}

String getGPSdataold() {

  String GPS_data = "";

  // Parse the raw GPS GGA data by detecting the beginning $
  if (GPSSerial.available() && GPSSerial.peek() == '$') {
    // while(GPSSerial.available()){
    for (int i = 0; i<70; i++){
      char c = GPSSerial.read();
      GPS_data += c;
      delay(5);
    }
  }  
  return GPS_data;
}

String getGPSdata() {
  String GPS_data = "";

  // Check if there's data available and if the next character is the beginning of an NMEA sentence ('$')
  // if (GPSSerial.available() && GPSSerial.read() == '$') {
    // Read characters until a newline character is encountered
  while (GPSSerial.available()) {
    char c = GPSSerial.read();
    GPS_data += c;
    delay(10);
    if (c == '\n') {
      break; // Exit the loop if a newline character is encountered
    }
  }
  // }

  return GPS_data;
}

// Define the GPS task function
void gpsTask(void *pvParameters) {
    // Initialize GPS module
    initLudacGPS();

    while (true) {
        // Your GPS library may have functions to read and process data
        String gpsData = getGPSdata();
        // Print GPS data
        Serial.println(gpsData);
        
        // Delay for a short duration before the next iteration
        delay(1000); // Adjust the delay as needed
    }
}

/**
 * @brief Get latitude and longitude from GPS fix.
 * 
 * @param latLong Pointer to store latitude, longitude, and timestamp.
 */
void getLatLong(struct LatLong *latLong) {
  if (GPS.fix) {
    latLong->latitude = convertDMtoDecimalDegrees(GPS.latitude);
    latLong->longitude = convertDMtoDecimalDegrees(GPS.longitude);
    latLong->hasValidFix = true;
    latLong->timestamp = millis();
  }

  // Check if latitude is in North or South hemisphere
  if (String(GPS.lat) == "N") {
    latLong->latitude *= 1;
  } else {
    latLong->latitude *= -1;
  }

  // Check if longitude is in East or West hemisphere
  if (String(GPS.lon) == "E") {
    latLong->longitude *= 1;
  } else {
    latLong->longitude *= -1;
  }
}

/**
 * @brief Check if a new GPS fix is available.
 * 
 * @param prevLatLong Pointer to previous GPS fix.
 * @param currLatLong Pointer to current GPS fix.
 * @return True if a new fix is available, false otherwise.
 */
bool hasNewGPSFix(struct LatLong *prevLatLong, struct LatLong *currLatLong) {
  if (!currLatLong->hasValidFix) {
    return false;
  }

  if (prevLatLong->latitude == currLatLong->latitude &&
      prevLatLong->longitude == currLatLong->longitude) {
    return false;
  }

  return true;
}

/**
 * @brief Get GPS time in HH:MM:SS format.
 * 
 * @param value String reference to store GPS time.
 */
void getGPStime(String &value) {
  uint8_t hourInTimezone = (GPS.hour + timezone) % 24;

  if (hourInTimezone < 10) {
    value = '0' + String(hourInTimezone, DEC);
  } else {
    value = String(hourInTimezone, DEC);
  }

  value += ':';

  if (GPS.minute < 10) {
    value += '0';
  }
  value += String(GPS.minute, DEC);
  value += ':';

  if (GPS.seconds < 10) {
    value += '0';
  }
  value += String(GPS.seconds, DEC);
  value += '.';

  // Convert milliseconds to a string before appending
  String millisString = String(GPS.milliseconds, DEC);
  if (GPS.milliseconds < 10) {
    value += "00" + millisString;
  } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
    value += "0" + millisString;
  } else {
    value += millisString;
  }
}

/**
 * @brief Get GPS date in YYYY-MM-DD format.
 * 
 * @param value String reference to store GPS date.
 */
void getGPSdate(String &value) {
  value = "20";
  value += String(GPS.year, DEC);
  value += '-';
  value += String(GPS.month, DEC);
  value += '-';
  value += String(GPS.day, DEC);
}

/**
 * 
 * @return True if GPS has a fix, false otherwise.
 */
bool getGPSfix() {
  return GPS.fix;
}

/**
 * @brief Get GPS fix quality.
 * 
 * @return GPS fix quality.
 */
uint8_t getGPSfixquality() {
  return GPS.fixquality;
}

/**
 * @brief Get GPS speed in meters per second.
 * 
 * @return GPS speed.
 */
float getGPSspeed() {
  return GPS.speed;
}

/**
 * @brief Get GPS angle in degrees.
 * 
 * @return GPS angle.
 */
float getGPSangle() {
  return GPS.angle;
}

/**
 * @brief Get GPS altitude in meters.
 * 
 * @return GPS altitude.
 */
float getGPSaltitude() {
  return GPS.altitude;
}

/**
 * @brief Get number of satellites used in GPS fix.
 * 
 * @return Number of satellites.
 */
uint8_t getGPSsatellites() {
  return GPS.satellites;
}

/**
 * @brief Get time since last GPS fix in seconds.
 * 
 * @return Time since last fix.
 */
float getGPStimeSinceLastFix() {
  return GPS.secondsSinceFix();
}

/**
 * @brief Get time since last GPS time update in seconds.
 * 
 * @return Time since last time update.
 */
float getGPSlastTime() {
  return GPS.secondsSinceTime();
}

/**
 * @brief Get time since last GPS date update in seconds.
 * 
 * @return Time since last date update.
 */
float getGPSlastDate() {
  return GPS.secondsSinceDate();
}

/**
 * @brief Convert latitude and longitude for display.
 * 
 * @param latLong Pointer to latitude, longitude, and fix status.
 * @param value String reference to store converted value.
 */
void convertLatLongForDisplay(struct LatLong *latLong, String &value) {
  String latitude = String(latLong->latitude, 2);
  String latitude_direction = latLong->latitude < 0 ? "S" : "N";

  String longitude = String(latLong->longitude, 2);
  String longitude_direction = latLong->longitude < 0 ? "W" : "E";

  value = latitude + latitude_direction + ", " + longitude + longitude_direction;
}

/**
 * @brief Convert latitude and longitude to string format.
 * 
 * @param latLong Pointer to latitude and longitude.
 * @param value String reference to store converted value.
 */
void convertLatLongToString(struct LatLong *latLong, String &value) {
  String latitude = String(latLong->latitude, 8);
  String longitude = String(latLong->longitude, 8);

  value = latitude + "," + longitude;
}

/**
 * @brief Calculate Haversine distance between two points.
 * 
 * @param latLong1 Pointer to first latitude-longitude point.
 * @param latLong2 Pointer to second latitude-longitude point.
 * @param haversine Pointer to store distance and time difference.
 */
void getHaversineDistance(struct LatLong *latLong1, struct LatLong *latLong2, struct Haversine *haversine) {
  // Calculate Haversine distance between two latitude-longitude points
  float lat1 = latLong1->latitude;
  float lon1 = latLong1->longitude;
  float lat2 = latLong2->latitude;
  float lon2 = latLong2->longitude;

  float dLat = (lat2 - lat1) * DEG_TO_RAD;
  float dLon = (lon2 - lon1) * DEG_TO_RAD;
  float lat1Rad = lat1 * DEG_TO_RAD;
  float lat2Rad = lat2 * DEG_TO_RAD;

  float a = sin(dLat / 2) * sin(dLat / 2) +
            sin(dLon / 2) * sin(dLon / 2) * cos(lat1Rad) * cos(lat2Rad);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float d = EARTH_RADIUS * c;

  haversine->distance = d;  // in km

  // Calculate time difference between 2 GPS fixes
  int localTimeDifference = abs(static_cast<long>(millis() - latLong1->timestamp));
  int peerTimeDifference = abs(static_cast<long>(millis() - latLong2->timestamp));

  int timeDifference = min(peerTimeDifference, localTimeDifference) / 1000;

  haversine->timeDiff = timeDifference;  // in seconds
}

/**
 * @brief Convert string format latitude and longitude to LatLong structure.
 * 
 * @param data String containing latitude and longitude.
 * @param latLong Pointer to store latitude, longitude, and fix status.
 */
void convertStringToLatLong(String data, struct LatLong *latLong) {
  int commaIndex = data.indexOf(",");
  int dataLength = data.length();

  String latitude = data.substring(0, commaIndex);
  String longitude = data.substring(commaIndex + 1, dataLength);

  latLong->latitude = latitude.toFloat();
  latLong->longitude = longitude.toFloat();
  latLong->hasValidFix = true;
  latLong->timestamp = millis();
}

/**
 * @brief Convert Degree-Minutes to Decimal Degrees.
 * 
 * @param value Degree-Minutes value to convert.
 * @return Decimal Degrees value.
 */
float convertDMtoDecimalDegrees(float value) {
  int size = String(value, 8).length();

  String minutes = String(value, 8).substring(size - 8 - 3, size);
  String degrees = String(value, 8).substring(0, size - 8 - 3);

  float decimalDegrees = degrees.toFloat() + minutes.toFloat() / 60;

  return decimalDegrees;
}

/**
 * @brief Get time difference in seconds between two timestamps.
 * 
 * @param peerTimestamp Timestamp of the peer node.
 * @param localTimestamp Timestamp of the local node.
 * @return Time difference in seconds.
 */
int getTimeDiff(uint32_t peerTimestamp, uint32_t localTimestamp) {
  int peerTimeDifference = abs(static_cast<long>(millis() - peerTimestamp));
  int localTimeDifference = abs(static_cast<long>(millis() - localTimestamp));

  int timeDifference = min(peerTimeDifference, localTimeDifference) / 1000;

  return timeDifference;
}

/**
 * @brief Check if it is okay to calculate Haversine distance.
 * 
 * @param peerLL Pointer to peer node's latitude, longitude, and fix status.
 * @param localLL Pointer to local node's latitude,
 * longitude, and fix status.
 * @return True if calculation is okay, false otherwise.
 */
bool isOKtoCalculateHaversine(struct LatLong *peerLL, struct LatLong *localLL) {
  if (peerLL->hasValidFix) {
    if (abs(static_cast<long>(peerLL->timestamp - localLL->timestamp)) < TEN_SECONDS) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Calculate time difference since given timestamp.
 * 
 * @param timestamp Reference timestamp.
 * @return Time difference in seconds.
 */
int calculateTimeDiff(uint32_t timestamp) {
  return (millis() - timestamp) / 1000;  // in seconds
}