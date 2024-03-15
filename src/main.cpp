#include <Arduino.h>
#include "LUDAC_LoRa.h"
#include "LUDAC_WiFi.h"
#include "LUDAC_GPS.h"

// Define verbose printing macro for debugging
#define VERBOSE_PRINT(x) \
    Serial.print(millis()); \
    Serial.print(": "); \
    Serial.println(x);

// Function prototype for printGPSInfo
void printGPSInfo();

// Declare variables
byte localAddress = 0xAA;
byte destinationAddress = 0xBB;

int sendLoRaInterval = 3000;  // Send LoRa packet every 3 seconds
uint32_t lastLoRaSendTime = 0;

int displayInterval = 1000;  // Display update interval
uint32_t lastDisplayTime = 0;

String dataFromDestinationAddress = "";

String gpsTime = "00:00";
String gpsDate = "2024-03-14";

String gpsLatLong = "waiting for fix";  // Example: "1234.12345678N, 12345.12345678E"
String gpsLatLongForDisplay = "";       // Example: "1 40N, 103 91E";

LatLong latLong = {0.00, 0.00, false};
LatLong prevLatLong = {0.00, 0.00, false};
LatLong peerLatLong = {0.00, 0.00, false};
Haversine haversine = {0.000, 0};

void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    delay(1000);

    // Initialize LoRa module
    VERBOSE_PRINT("Starting LUDAC " + String(localAddress, HEX));
    VERBOSE_PRINT("   localAddress: " + String(localAddress, HEX));
    VERBOSE_PRINT("   destinationAddress: " + String(destinationAddress, HEX));

    if (!initLoRa()) {
        VERBOSE_PRINT("Starting LoRa failed!");
    } else {
        VERBOSE_PRINT("Starting LoRa succeeded!");
    }

    if (!initLudacWIFI()) {
        VERBOSE_PRINT("Starting WiFI failed!");
    } else {
        VERBOSE_PRINT("Starting WiFI succeeded!");
    }

    // Initialize GPS module
    VERBOSE_PRINT("Initializing GPS ...");
    initGPS();
}

void loop() {
    // Send LoRa packet if interval has passed
    if (millis() - lastLoRaSendTime > sendLoRaInterval) {
        if (hasNewGPSFix(&prevLatLong, &latLong)) {
            sendLoRaString(gpsLatLong, localAddress, destinationAddress);
            VERBOSE_PRINT("Sent data " + gpsLatLong + " from 0x" + String(localAddress, HEX) + " to 0x" + String(destinationAddress, HEX));
            lastLoRaSendTime = millis();
        }
    }

    // Receive LoRa packet
    if (receiveLoRaString(LoRa.parsePacket(), localAddress, dataFromDestinationAddress)) {
        VERBOSE_PRINT("Received data " + dataFromDestinationAddress + " from 0x" + String(destinationAddress, HEX) + " to 0x" + String(localAddress, HEX));
        convertStringToLatLong(dataFromDestinationAddress, &peerLatLong);
    }

    // Process GPS data
    if (receivedGPSfix()) {
    // if (true) {
        getGPStime(gpsTime);
        getGPSdate(gpsDate);
        getLatLong(&latLong);

        // Print GPS information
        printGPSInfo();

        // Display data if interval has passed
        if (millis() - lastDisplayTime > displayInterval) {
            if (hasNewGPSFix(&prevLatLong, &latLong)) {
                convertLatLongForDisplay(&latLong, gpsLatLongForDisplay);

                if (isOKtoCalculateHaversine(&latLong, &peerLatLong)) {
                    getHaversineDistance(&latLong, &peerLatLong, &haversine);
                    VERBOSE_PRINT("Haversine distance: " + String(haversine.distance, 3) + " km " + String(haversine.timeDiff, DEC) + "s ago");
                } else {
                    if (peerLatLong.hasValidFix) {
                        int timeDiff = calculateTimeDiff(peerLatLong.timestamp);
                        VERBOSE_PRINT("CANNOT calculate Haversine distance.");
                        VERBOSE_PRINT("Haversine distance: " + String(haversine.distance, 3) + " km");
                        VERBOSE_PRINT("Haversine time difference: " + String(timeDiff, DEC) + "s ago");
                    } else {
                        VERBOSE_PRINT(gpsLatLongForDisplay + " searching for peer");
                    }
                }

                prevLatLong = latLong;
            } 
            else {
                VERBOSE_PRINT("No GPS fix yet. Date: " + gpsDate + " Time: " + gpsTime + " Lat/Long: " + gpsLatLong);
            }

            lastDisplayTime = millis();
        }
    }
}

// Print GPS information
void printGPSInfo() {
    VERBOSE_PRINT("----------------------------------------");
    VERBOSE_PRINT("Date: " + gpsDate);
    VERBOSE_PRINT("Time: " + gpsTime);
    convertLatLongToString(&latLong, gpsLatLong);
    VERBOSE_PRINT("Lat/Long: " + gpsLatLong);
    VERBOSE_PRINT("GPS Fix: " + String(getGPSfix(), DEC));
    VERBOSE_PRINT("GPS Fix Quality: " + String(getGPSfixquality(), DEC));
    VERBOSE_PRINT("Speed (knots): " + String(getGPSspeed()));
    VERBOSE_PRINT("Angle: " + String(getGPSangle()));
    VERBOSE_PRINT("Altitude: " + String(getGPSaltitude()));
    VERBOSE_PRINT("Satellites: " + String(getGPSsatellites()));
    VERBOSE_PRINT("Time [s] since last fix: " + String(getGPStimeSinceLastFix(), 3));
    VERBOSE_PRINT("    since last GPS time: " + String(getGPSlastTime(), 3));
    VERBOSE_PRINT("    since last GPS date: " + String(getGPSlastDate(), 3));
}

// DO NOT USE !!!
// WORK IN PROGRESS (if GPS coordinates send by LoRa char (not string) packets)

// void ludac_lora_loop() {
  
//   if(GPS_serial.available()){
    

//     // Convert GPS lat, lon, and distance to the corresponding char arrays
//     char lat[15] = {0};
//     sprintf(lat, "%f", fake_gps_lat); //change this to GPS.latitude in practise
    
//     char lon[15] = {0};
//     sprintf(lon, "%f", fake_gps_lon); //change this to GPS.longtitude in practise

//     // call the relaDistance function to calculate the relative distance
//     float dist = relaDistance(fake_gps_lat, fake_gps_lon, 53.534940, -113.541585);
    
//     char dis[15] = {0};
//     sprintf(dis, "%f", dist);
    
//     // Insert the lat, lon, and distance into the buffer array
//     for(int j = 0; j<15; j++){
//       LoRa_sending_buffer[j] = lat[j];
//     }
    
//     for(int k = 0; k<15; k++){
//       LoRa_sending_buffer[k+15] = lon[k];
//     }
      
//     for(int z = 0; z<15; z++){
//       LoRa_sending_buffer[z+30] = dis[z];
//     }
    
//   }

//   // If the difference between current and the last send time is greater than the interval time, 
//   // send the buffered data
//   if (millis() - lastSendTime > interval)
//   {
//     sendMSG(LoRa_sending_buffer);
    
//     lastSendTime = millis();
//     interval = random(2000) + 1000;
//   }

//   // When not sending LoRa packets, listen to the incoming messages
//   onReceive(LoRa.parsePacket());
// }