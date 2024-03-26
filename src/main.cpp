#include <Arduino.h>
#include <HardwareSerial.h>
#include "LUDAC_LoRa.h"
#include "LUDAC_WiFi.h"
#include "LUDAC_GPS.h"
#include "driver/uart.h"

#define UART_LUDAC_BAUD 9600
#define UART_RX2_RaspPi 16
#define UART_TX2_RaspPi 17
#define UART_RaspPi_Enable 0
#define UART_RaspPi_BAUD 9600
#define UART_RaspPi_ENC SERIAL_8N1
#define TRANSMIT_MASTER_TO_SLAVE_EN 'T'
#define TRANSMIT_SLAVE_TO_MASTER_EN 'R'

// Define verbose printing macro for debugging
#define VERBOSE_PRINT(x) \
    Serial.print(millis()); \
    Serial.print(": "); \
    Serial.println(x);

// Function prototype for printGPSInfo
void printGPSInfo();
void parseGGA(const String& ggaData, LatLong& latLong);
char* generate_LoRaTx_Buffer(const char* gpsBuffer_Tx, const char* uartDataReceivedFromMaster);
bool Parse_LoRaRx_Buffer(const char* receive_LoRa_Buffer, char* gpsBuffer_Rx, char* LoRa_Data_For_Master);
String convertUTCToMST(const String& utcTime);

// Initialize Serial connection
HardwareSerial Serial_UART_RaspPi(2); // Use Serial2 on ESP32

// Define initial buffer size and maximum buffer size
const size_t initialBufferSize = 64;
const size_t maxBufferSize = 100000;
char *uartDataReceivedFromMaster = nullptr;
size_t currentBufferSize = 0;

// Define a global variable to hold the received LoRa data
char* receive_LoRa_Buffer = nullptr;

// Declare variables
byte localAddress = 0xAA;
byte destinationAddress = 0xBB;

int sendTxInterval = 3000;  // Send LoRa/WiFi packet bundle every 3 seconds
uint32_t lastTxSendTime = 0;

int displayInterval = 1000;  // Display update interval
uint32_t lastDisplayTime = 0;

unsigned long lastRSSICheckTime = 0;
const unsigned long checkRSSIInterval = 1000; // 1 seconds

int WiFi_RSSI, LoRa_RSSI = -1;
bool wifiEnabled, loraEnabled;

char gpsBuffer_Tx[70] = {}; // 70 characters fixed
char gpsBuffer_Rx[70] = {}; // 70 characters fixed
char LoRa_Data_For_Master[180] = {};

String dataFromDestinationAddress = "";

String gpsTime = "00:00";
String gpsDate = "2024-03-14";

String gpsLatLong = "waiting for fix";  // Example: "1234.12345678N, 12345.12345678E"
String gpsLatLongForDisplay = "";       // Example: "1 40N, 103 91E";

LatLong currentLocation;
// LatLong latLong = {0.00, 0.00, 0.00, 0, 00:00:00:00, false};
// LatLong prevLatLong = {0.00, 0.00, 0.00, 0, 00:00:00:00, false};
// LatLong peerLatLong = {0.00, 0.00, 0.00, 0, 00:00:00:00, false};
// Haversine haversine = {0.000, 0};

void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    delay(1000);

    // Initialize LoRa module
    VERBOSE_PRINT("Starting LUDAC " + String(localAddress, HEX));
    VERBOSE_PRINT("   localAddress: " + String(localAddress, HEX));
    VERBOSE_PRINT("   destinationAddress: " + String(destinationAddress, HEX));

    // Initialize LoRa module
    bool loraInitSuccess = initLoRa();

    // Print initialization result
    VERBOSE_PRINT(loraInitSuccess ? "Starting LoRa succeeded!" : "Starting LoRa failed!");

    // // Initialize Wi-Fi module
    // bool wifiInitSuccess = initLudacWIFI();

    // // Print initialization result
    // VERBOSE_PRINT(wifiInitSuccess ? "Starting WiFI succeeded!" : "Starting WiFI failed!");

    // If Wi-Fi initialization succeeded, try to register with a peer device
    // if (wifiInitSuccess) {
    //     // Attempt to register with a peer device
    //     bool peerRegistrationSuccess = WiFi_RegisterPeer();

    //     // Print peer registration result
    //     VERBOSE_PRINT(peerRegistrationSuccess ? "Peer registration successful!" : "Peer registration failed!");
    // }

    // Initialize GPS module
    VERBOSE_PRINT("Initializing GPS ...");
    initGPS();

    // If UART communication with Raspberry Pi is enabled
    if (UART_RaspPi_Enable){
        VERBOSE_PRINT("Initializing UART Connection with External Board ...");
        
        // Begin serial communication
        Serial_UART_RaspPi.begin(UART_RaspPi_BAUD, UART_RaspPi_ENC, UART_RX2_RaspPi, UART_TX2_RaspPi);

        // Allocate memory for the initial buffer size
        uartDataReceivedFromMaster = new char[initialBufferSize];
        currentBufferSize = initialBufferSize;
    }
}


void loop1(){

    String printGPSI = getGPSdata();
    VERBOSE_PRINT(printGPSI);
        
    // String ggaData = "$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F"; // Test Data
    parseGGA(printGPSI, currentLocation);
    printGPSInfo();

    Serial.println(getWiFiRSSI());
    Serial.println(getLoRaRSSI());

    delay(500);
}

void loop() {
    
    // If receive_LoRa_Buffer is already declared, delete it to avoid memory leaks
    if (receive_LoRa_Buffer != nullptr) {
        delete[] receive_LoRa_Buffer;
        receive_LoRa_Buffer = nullptr;
    }

    // Allocate memory for receive_LoRa_Buffer
    receive_LoRa_Buffer = new char[250];

    if (UART_RaspPi_Enable && Serial_UART_RaspPi.available()) {
      char RaspPi_instr = Serial_UART_RaspPi.read();

      if (RaspPi_instr == TRANSMIT_MASTER_TO_SLAVE_EN) {
          size_t bytesRead = 0;
          size_t bufferIndex = 0;

          // Read data until newline character '\n'
          while (Serial_UART_RaspPi.available() && bufferIndex < currentBufferSize - 1) {
              char receivedByte = Serial_UART_RaspPi.read();
              uartDataReceivedFromMaster[bufferIndex++] = receivedByte;

              if (receivedByte == '\n') {
                  // Process received data
                  uartDataReceivedFromMaster[bufferIndex] = '\0'; // Null-terminate the string
                  Serial.println("Received data from Raspberry Pi:");
                  Serial.println(uartDataReceivedFromMaster);
                  bytesRead = bufferIndex;

                  // Reset buffer index for the next transmission
                  bufferIndex = 0;
              }
          }

          // Check if buffer overflow occurred
          if (bufferIndex == currentBufferSize - 1) {
              Serial.println("Buffer overflow. Resizing buffer.");

              currentBufferSize = min(currentBufferSize * 2, maxBufferSize);
              char *tempBuffer = new char[currentBufferSize];
              memcpy(tempBuffer, uartDataReceivedFromMaster, bytesRead);

              delete[] uartDataReceivedFromMaster;
              uartDataReceivedFromMaster = tempBuffer;
          }
      }

      if (RaspPi_instr == TRANSMIT_SLAVE_TO_MASTER_EN){
        
        // Send gpsBuffer_Rx
        for (size_t i = 0; i < strlen(gpsBuffer_Rx); ++i) {
            Serial_UART_RaspPi.write(gpsBuffer_Rx[i]);
        }

        // Send LoRa_Data_For_Master
        for (size_t i = 0; i < strlen(LoRa_Data_For_Master); ++i) {
            Serial_UART_RaspPi.write(LoRa_Data_For_Master[i]);
        }
      }
    }

    // Get the current time
    unsigned long currentTime = millis();

    // Convert currentTime to a string
    char currentTimeStr[20]; // Assuming the maximum length of the converted string
    snprintf(currentTimeStr, sizeof(currentTimeStr), "%lu", currentTime);

    // Concatenate the string literal with currentTimeStr
    VERBOSE_PRINT("currentTime: " + String(currentTimeStr));
    // Check if it's time to update RSSI values
    if (currentTime - lastRSSICheckTime > checkRSSIInterval) {

        wifiEnabled, loraEnabled = false;

        VERBOSE_PRINT("Comparing the RSSI strength");
        // Update lastRSSICheckTime
        lastRSSICheckTime = currentTime;

        // Convert lastRSSICheckTime to a string
        char lastRSSICheckTimeStr[20]; // Assuming the maximum length of the converted string
        snprintf(lastRSSICheckTimeStr, sizeof(lastRSSICheckTimeStr), "%lu", lastRSSICheckTime);

        // Concatenate the string literal with lastRSSICheckTimeStr
        VERBOSE_PRINT("lastRSSICheckTime: " + String(lastRSSICheckTimeStr));

        // Determine if Wi-Fi and LoRa are enabled based on RSSI values
        wifiEnabled = (getWiFiRSSI() != -1 && (getLoRaRSSI() == -1 || WiFi_RSSI > LoRa_RSSI));
        loraEnabled = (getLoRaRSSI() != -1 && (getWiFiRSSI() == -1 || LoRa_RSSI > WiFi_RSSI));

        VERBOSE_PRINT(wifiEnabled);
        VERBOSE_PRINT(loraEnabled);

        // If neither Wi-Fi nor LoRa has a valid RSSI value, keep both off
        if (WiFi_RSSI == -1 && LoRa_RSSI == -1) {
            wifiEnabled = false;
            loraEnabled = false;
        } else if (wifiEnabled && loraEnabled) {
            // Ensure only one of Wi-Fi or LoRa is enabled, or both are off
            if (WiFi_RSSI > LoRa_RSSI) {
                loraEnabled = false;
            } else {
                wifiEnabled = false;
            }
        }
    }

    if (currentTime - lastTxSendTime > sendTxInterval) {

      if (loraEnabled){

        VERBOSE_PRINT("LoRa Enabled - Working ... ");

        // Call generate_LoRaTx_Buffer to create transmit_LoRa_Buffer
        char* transmit_LoRa_Buffer = generate_LoRaTx_Buffer(gpsBuffer_Tx, uartDataReceivedFromMaster);
        
        if (transmit_LoRa_Buffer == nullptr) {
          // Handle error
          // Maybe print an error message or take appropriate action
          VERBOSE_PRINT("Error: Failed to generate LoRa transmit buffer");
          return;
        }

        // Ensure transmit_LoRa_Buffer is not larger than 250 bytes
        transmit_LoRa_Buffer[std::min(strlen(transmit_LoRa_Buffer), size_t(250))] = '\0';
        VERBOSE_PRINT("ok8");

        // Now call sendLoRaChar with transmit_LoRa_Buffer
        sendLoRaChar(transmit_LoRa_Buffer, 250, localAddress, destinationAddress);

        VERBOSE_PRINT("Sending data from 0x" + String(destinationAddress, HEX) + " to 0x" + String(localAddress, HEX));
        VERBOSE_PRINT("ok9");
        
        lastTxSendTime = millis();

        receive_LoRa_Buffer = receiveLoRaChar(LoRa.parsePacket(), localAddress);
        VERBOSE_PRINT("Received data from 0x" + String(destinationAddress, HEX) + " to 0x" + String(localAddress, HEX));

        Parse_LoRaRx_Buffer(receive_LoRa_Buffer, gpsBuffer_Rx, LoRa_Data_For_Master);
        VERBOSE_PRINT("ok11");

          // Free the memory allocated for the receive buffer
        delete[] receive_LoRa_Buffer;
        receive_LoRa_Buffer = nullptr;
      }

      if (wifiEnabled){
        
        VERBOSE_PRINT("WiFi Enabled - Idling ... ");
        
        delete[] receive_LoRa_Buffer;
        receive_LoRa_Buffer = nullptr;

        delay(1000);
      }
      
    for (int tryCount = 0; tryCount < 10 && !currentLocation.hasValidFix; tryCount++){
      
      // Reset struct to default
      currentLocation.reset();
      // Reset buffer to all zeros
      memset(gpsBuffer_Tx, 0, sizeof(gpsBuffer_Tx));

      String currentGPSRawData = getGPSdata();

      VERBOSE_PRINT("check1");
      VERBOSE_PRINT(currentGPSRawData);

      currentGPSRawData.toCharArray(gpsBuffer_Tx, sizeof(gpsBuffer_Tx));
      
      VERBOSE_PRINT("check2");
      VERBOSE_PRINT(currentGPSRawData);
      parseGGA(currentGPSRawData, currentLocation);

      delay(500);
    }

    if (0){
        String GPSdata = getGPSdata();

        VERBOSE_PRINT(GPSdata);
        
        // testdata
        String ggaData = "$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F";
        parseGGA(GPSdata, currentLocation);
        printGPSInfo();

        Serial.println(getWiFiRSSI());
        Serial.println(getLoRaRSSI());

        delay(500);
    }

    // lastRSSICheckTime = currentTime;
  }
}

char* generate_LoRaTx_Buffer(const char* gpsBuffer_Tx, const char* uartDataReceivedFromMaster) {
    // Check if uartDataReceivedFromMaster is nullptr
    if (uartDataReceivedFromMaster == nullptr) {

        // Check if both gpsBuffer_Tx and uartDataReceivedFromMaster are nullptr
        if (gpsBuffer_Tx == nullptr) {
            // Handle the case where both gpsBuffer_Tx and uartDataReceivedFromMaster are nullptr
            static char defaultBuffer[251] = "No UART data received";
            // Fill the rest of the buffer with null characters
            memset(defaultBuffer + strlen(defaultBuffer), '\0', sizeof(defaultBuffer) - strlen(defaultBuffer));
            return defaultBuffer;
        }
        // If uartDataReceivedFromMaster is not nullptr, fill the buffer with gpsBuffer_Tx
        // or return nullptr if gpsBuffer_Tx is nullptr
        return (gpsBuffer_Tx != nullptr) ? strdup(gpsBuffer_Tx) : nullptr;
    }

    size_t uartDataReceivedFromMasterLength = strlen(uartDataReceivedFromMaster);
    VERBOSE_PRINT("oka");
    // Calculate the maximum length that can be copied while ensuring it doesn't exceed 250 bytes
    size_t max_gps_length = std::min(strlen(gpsBuffer_Tx), size_t(70));
    size_t max_uart_length = std::min(uartDataReceivedFromMasterLength - 1, size_t(250 - max_gps_length - 1)); // -1 for null terminator
    VERBOSE_PRINT("oks");
    
    // Allocate memory for transmit_LoRa_Buffer
    static char transmit_LoRa_Buffer[251]; // Static buffer of 250 bytes
    VERBOSE_PRINT("okd");

    // Copy gpsBuffer_Tx to transmit_LoRa_Buffer
    memcpy(transmit_LoRa_Buffer, gpsBuffer_Tx, max_gps_length);
    VERBOSE_PRINT("okf");

    // Copy uartDataReceivedFromMaster to transmit_LoRa_Buffer if there's space
    memcpy(transmit_LoRa_Buffer + max_gps_length, uartDataReceivedFromMaster, max_uart_length);
    VERBOSE_PRINT("okg");

    // Null-terminate the string
    size_t totalLength = max_gps_length + max_uart_length;
    transmit_LoRa_Buffer[totalLength] = '\0';
    VERBOSE_PRINT("okh");
    return transmit_LoRa_Buffer;
}

bool Parse_LoRaRx_Buffer(const char* receive_LoRa_Buffer, char* gpsBuffer_Rx, char* LoRa_Data_For_Master) {
    // Calculate the length of transmit_LoRa_Buffer
    if (receive_LoRa_Buffer == nullptr){

      VERBOSE_PRINT("parse1exit");

      gpsBuffer_Rx = nullptr;
      LoRa_Data_For_Master = nullptr;
      return false;
    }
    size_t totalLength = strlen(receive_LoRa_Buffer);
    VERBOSE_PRINT("parse1");

    // Determine the length of data to copy into gpsBuffer_Rx
    size_t gpsLength = std::min(totalLength, size_t(70));
    memcpy(gpsBuffer_Rx, receive_LoRa_Buffer, gpsLength);
    gpsBuffer_Rx[gpsLength] = '\0'; // Null-terminate the string

    // Determine the length of data to copy into LoRa_Data_For_Master
    size_t uartLength = std::min(totalLength - gpsLength, size_t(180));
    memcpy(LoRa_Data_For_Master, receive_LoRa_Buffer + gpsLength, uartLength);
    LoRa_Data_For_Master[uartLength] = '\0'; // Null-terminate the string

    VERBOSE_PRINT("parse2");

    return true;
}

// Print GPS information for debug
void printGPSInfo() {
    VERBOSE_PRINT("----------------------------------------");
    VERBOSE_PRINT("UTC Time: " + convertUTCToMST(currentLocation.utcTime));
    VERBOSE_PRINT("Latitude: " + String(currentLocation.latitude, 8));
    VERBOSE_PRINT("Longitude: " + String(currentLocation.longitude, 8));
    VERBOSE_PRINT("Has Valid Fix: " + String(currentLocation.hasValidFix));
    VERBOSE_PRINT("Altitude: " + String(currentLocation.altitude));
    VERBOSE_PRINT("GPS Quality: " + String(currentLocation.gpsQuality));
    VERBOSE_PRINT("Timestamp: " + String(currentLocation.timestamp));
}

// Function to convert UTC time to MST
String convertUTCToMST(const String& utcTime) {
  int hour = utcTime.substring(0, 2).toInt();
  hour = (hour + 17) % 24; // Apply UTC-7 offset for MST
  String mstTime = String(hour) + utcTime.substring(2); // Retain minutes and seconds
  return mstTime;
}

// Function to parse GPS GGA data
void parseGGA(const String& ggaData, LatLong& latLong) {
  // Check if the GGA sentence starts with "$GPGGA"
  if (ggaData.startsWith("$GPGGA")) {
    // Split the GGA sentence by commas
    int commasFound = 0;
    int startPos = 0;
    int endPos = -1;

    // Extract latitude, longitude, and fix status
    int commaCount = std::count_if(ggaData.begin(), ggaData.end(), [](char c) { return c == ','; });
    while (commasFound < commaCount) {
      endPos = ggaData.indexOf(',', startPos);
      if (endPos != -1) {
        switch (commasFound) {
          case 1: {
            // Extract UTC time
            String timeStr = ggaData.substring(startPos, endPos);
            latLong.utcTime = timeStr.substring(0, 2) + ":" + timeStr.substring(2, 4) + ":" + timeStr.substring(4);
            break;
          }
          case 2: {
            // Extract latitude
            latLong.latitude = ggaData.substring(startPos, endPos).toFloat();
            break;
          }
          case 3: {
            // Extract latitude direction (N/S)
            if (ggaData.charAt(endPos + 1) == 'S') {
              latLong.latitude *= -1.0;
            }
            break;
          }
          case 4: {
            // Extract longitude
            latLong.longitude = ggaData.substring(startPos, endPos).toFloat();
            break;
          }
          case 5: {
            // Extract longitude direction (E/W)
            if (ggaData.charAt(endPos + 1) == 'W') {
              latLong.longitude *= -1.0;
            }
            break;
          }
          case 6: {
            // Extract GPS quality indicator
            latLong.gpsQuality = ggaData.substring(startPos, endPos).toInt();
            break;
          }
          case 9: {
            // Extract altitude
            latLong.altitude = ggaData.substring(startPos, endPos).toFloat();
            break;
          }
        }
        startPos = endPos + 1;
        commasFound++;
      }
    }

    // Extract fix status
    latLong.hasValidFix = (latLong.gpsQuality > 0);
    
    // Extract timestamp (optional)
    endPos = ggaData.indexOf(',', startPos);
    if (endPos != -1) {
      String timestampStr = ggaData.substring(startPos, endPos);
      latLong.timestamp = timestampStr.toFloat() * 1000; // Convert to milliseconds
    }
  }
}
