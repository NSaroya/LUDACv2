// ======================================================================================
// NAME
//   main.cpp
//
// DESCRIPTION
//   Main firmware for the LUDAC (Long-distance Underwater Drone to Aerial Communication)
//   ECE 490/491 Capstone Design Project
//
// AUTHOR
//   April 2024    Nikhil Saroya 
//                 main.cpp, LUDAC_GPS.cpp, LUDAC_WiFi.cpp
//   April 2024    Xinkai Xu
//                 LUDAC_LoRa.cpp, LUDAC_GPS.cpp
//   April 2024    Ethan Wilson - 
//                 LUDAC_WiFi_Duplex.cpp, LUDAC_WiFi.cpp
//
// TEAM MEMBERS
//   Nikhil Saroya, Xinkai Xu, Ethan Wilson, Antony Brittain
//
// FUNCTIONS
//    setup, loop, GetPeerRSSI, generate_LoRaTx_Buffer, Parse_LoRaRx_Buffer, printGPSInfo
//    convertUTCToMST, parseGGA
//
// NOTES
//  Ensure most up-to-date Config files used in conjunction with this script
//
// ======================================================================================

#include <Arduino.h>
#include <HardwareSerial.h>
#include "LUDAC_LoRa.h"
#include "LUDAC_WiFi.h"
#include "LUDAC_GPS.h"
#include "driver/uart.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define UART_LUDAC_BAUD 9600
#define UART_RX2_RaspPi 16
#define UART_TX2_RaspPi 17
#define UART_RaspPi_Enable 0
#define UART_RaspPi_BAUD 9600
#define UART_RaspPi_ENC SERIAL_8N1
#define TRANSMIT_MASTER_TO_SLAVE_EN 'T'
#define TRANSMIT_SLAVE_TO_MASTER_EN 'R'
#define EnableDualCoreforGPS false
#define GPIO_EVENT_TRIGGER 22 // Define a GPIO for digital writes
volatile bool busy = false;

// Define verbose printing macro for debugging
#define VERBOSE_PRINT(x) \
    Serial.print(millis()); \
    Serial.print(": "); \
    Serial.println(x);

// Function prototypes
void GetPeerRSSI(const uint8_t *peerMAC);
void printGPSInfo();
void parseGGA(String& ggaData, LatLong& latLong);
char* generate_LoRaTx_Buffer_Dynamic(const char* gpsBuffer_Tx, const char* uartDataReceivedFromMaster);
char* generate_LoRaTx_Buffer_Static(const char* gpsBuffer_Tx, const char* uartDataReceivedFromMaster);
bool Parse_LoRaRx_Buffer(const char* receive_LoRa_Buffer, char* gpsBuffer_Rx, char* LoRa_Data_For_Master);
String convertUTCToMST(const String& utcTime);

// Initialize Serial connection
HardwareSerial Serial_UART_RaspPi(2); // Use Serial2 on ESP32

// Define initial buffer size and maximum buffer size
const size_t initialBufferSize = 64;
const size_t maxBufferSize = 100000;
char *uartDataReceivedFromMaster = nullptr;
size_t currentBufferSize = 0;

// Declare variables
byte localAddress = 0xBB;
byte destinationAddress = 0xAA;

// Define a global variable to hold the received LoRa data
char* receive_LoRa_Buffer = nullptr;

int sendLoRaTxInterval = 1000;  // Send LoRa packet bundle every 1 seconds
uint32_t lastLoRaTxInterval = 0;
bool firstTimeLoRaListening = true; // Initialize the flag to true initially

unsigned long lastRSSICheckTime = 0;
const unsigned long checkRSSIInterval = 60000; // 60 seconds

int8_t WiFi_RSSI = 0;
int LoRa_RSSI = 0;
bool wifiEnabled = false; // default wifi enabled
bool loraEnabled = false;
bool wifi_init = false;
bool lora_init = false;
bool isFirstCheck = true;

char gpsBuffer_Tx[70] = {}; // 70 characters fixed
char gpsBuffer_Rx[70] = {}; // 70 characters fixed
char LoRa_Data_For_Master[180] = {};

String currentGPSRawData = "";
char currentGPSRawDataBuffer[71] = {}; // Buffer to hold GPS data (including null terminator)

uint8_t broadcastAddress[] = {0xA0, 0xA3, 0xB3, 0x89, 0x23, 0xE4}; // weird WOER antenna
// uint8_t broadcastAddress[] = {0x40, 0x22, 0xD8, 0x08, 0xF9, 0x94}; // PATCH ANTENNA
  // {0x40, 0x22, 0xD8, 0x06, 0x75, 0x2C}, // MAC address of transceiver B (no sticker)
  // {0xB8, 0xD6, 0x1A, 0x67, 0xF8, 0x54}, // MAC address of transceiver C (covered microstrip antenna)
  // {0xA0, 0xA3, 0xB3, 0x89, 0x23, 0xE4}  // MAC address of transceiver D (weird WOER antenna, doesn't receive)

LatLong currentLocation;

// Structure to hold the peer information
typedef struct {
  int8_t rssi; // RSSI value of the peer
} PeerRSSI;

void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    delay(1000);

    // Initialize LoRa module
    VERBOSE_PRINT("Starting LUDAC " + String(localAddress, HEX));
    VERBOSE_PRINT("   localAddress: " + String(localAddress, HEX));
    VERBOSE_PRINT("   destinationAddress: " + String(destinationAddress, HEX));

    // Initialize LoRa module
    if (!initLudacLoRa()) {
        VERBOSE_PRINT("Starting LoRa failed!");
    } else {
        lora_init = true;
        VERBOSE_PRINT("Starting LoRa succeeded!");
    }

    // Initialize Wi-Fi module or register with a peer device
    if (initLudacWIFI()) {
        VERBOSE_PRINT("Starting WiFI in Progress...");
        WiFi_RegisterPeerManual(broadcastAddress);        
        WiFi_Packet_Handling_init();
        wifi_init = true;
        VERBOSE_PRINT("Starting WiFI succeeded!");        
    } else {
        VERBOSE_PRINT("Starting WiFI failed!");
    }

    // Initialize GPS module
    VERBOSE_PRINT("Initializing GPS ...");
    
    if (EnableDualCoreforGPS){
      // Create a task for GPS processing
      VERBOSE_PRINT("Enabling 2nd Core for GPS Task ...");
      // xTaskCreatePinnedToCore(taskGPS, "taskGPS", 4096, NULL, 1, NULL, 1); // Core 1
    } else {
      VERBOSE_PRINT("Enabling GPS Task with the Default Core...");
      initLudacGPS();
    }

    // If UART communication with Raspberry Pi is enabled
    if (UART_RaspPi_Enable){
        VERBOSE_PRINT("Initializing UART Connection with External Board ...");
        
        // Begin serial communication
        Serial_UART_RaspPi.begin(UART_RaspPi_BAUD, UART_RaspPi_ENC, UART_RX2_RaspPi, UART_TX2_RaspPi);

        pinMode(GPIO_EVENT_TRIGGER, OUTPUT);
        digitalWrite(GPIO_EVENT_TRIGGER, LOW); // Initially not ready
    }
}

void loop() {
    
    // If receive_LoRa_Buffer is already declared, delete it to avoid memory leaks
    if (receive_LoRa_Buffer != nullptr) {
        delete[] receive_LoRa_Buffer;
        receive_LoRa_Buffer = nullptr;
    }

    // If receive_LoRa_Buffer is already declared, delete it to avoid memory leaks
    if (uartDataReceivedFromMaster != nullptr) {
        delete[] uartDataReceivedFromMaster;
        uartDataReceivedFromMaster = nullptr;
    }

    if (UART_RaspPi_Enable && Serial_UART_RaspPi.available()) {

      // Allocate memory for the initial buffer size
      uartDataReceivedFromMaster = new char[initialBufferSize];
      currentBufferSize = initialBufferSize;

      // Allocate memory for receive_LoRa_Buffer
      receive_LoRa_Buffer = new char[250]; 
      
      // Signal readiness to Raspberry Pi
      digitalWrite(GPIO_EVENT_TRIGGER, HIGH);
      
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
              delete[] tempBuffer;
          }
      }

      if (RaspPi_instr == TRANSMIT_SLAVE_TO_MASTER_EN){
        
        if (loraEnabled && lora_init){
          // Send LoRa_received_buffer
          for (size_t i = 0; i < sizeof(LoRa_received_buffer); ++i) {
              Serial_UART_RaspPi.write(LoRa_received_buffer[i]);
          }

          if (wifiEnabled && wifi_init){
            // Concatenate GPS data and recvBuffer
            char combinedData[GPS_DATA_SIZE + recvBufferSize]; // Adjust the size as needed
            sprintf(combinedData, "%s%s", incomingPayload.GPS_data, recvBuffer);

            // Send combined data over UART to Raspberry Pi
            for (size_t i = 0; i < strlen(combinedData); ++i) {
                Serial_UART_RaspPi.write(combinedData[i]);
            }
          }
        }
      }
      digitalWrite(GPIO_EVENT_TRIGGER, LOW);
    } 

    // Get the current time
    unsigned long currentTime = millis();

    // Check if it's time to update RSSI values
    if (isFirstCheck || (currentTime - lastRSSICheckTime >= checkRSSIInterval)) {
        
      lastRSSICheckTime = millis();
      isFirstCheck = false;

      int success_count = 0;
      int test_packet_cnt = 20;
      VERBOSE_PRINT("Deciding the Wi-Fi transmission success rate");

      if (wifi_init){
        for (int iter = 0; iter < test_packet_cnt ; iter++){
          GetPeerRSSI(broadcastAddress);
          // Assuming success is a string
          if (success.equals("Delivery Success")) {
              success_count++;
          }
        }
      }

      // Calculate Wi-Fi transmission rate
      float transmission_rate = (float(success_count) / test_packet_cnt) * 100;
      VERBOSE_PRINT("Wi-Fi transmission rate - " + String(transmission_rate, 2) + "%");

      // Checking if more than 90% of the entries are "Delivery Success"
      if (success_count >= (0.9*test_packet_cnt)) { // 0.9 * 20
          wifiEnabled = true;
          loraEnabled = false;
          VERBOSE_PRINT("Wi-Fi is Enabled, LoRa is Disabled");
      } else if (lora_init) {
          loraEnabled = true;
          wifiEnabled = false;
          VERBOSE_PRINT("Wi-Fi is Disabled, LoRa is Enabled");
      } else {
          loraEnabled = false;
          wifiEnabled = false;
          VERBOSE_PRINT("Neither Wi-Fi or LoRa is Enabled");
      }
      delay(3000);   
      VERBOSE_PRINT("Starting Communication");
    }

    // Check if LoRa is enabled and initialized
    if (loraEnabled && lora_init){

      if (millis() - lastLoRaTxInterval > sendLoRaTxInterval){

        // Check if it's time to send LoRa transmission
        lastLoRaTxInterval = millis();
        sendLoRaTxInterval = random(5000) + 2000;

        // String currentGPSRawData = "$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F"; 
        currentGPSRawData = getGPSdata(); // Get current GPS data (commented out for testing purposes)

        // Example data to send via LoRa
        const char example_data_to_send_lora[] = " Submarine Status: Depth 300 meters, Speed 15 knots, Battery Level 85%. Nearby Vessel Detected: Type - Cargo Ship, Distance - 500 meters, Weather Conditions: Moderate Sea State.";
        
        // Convert GPS data to char array
        char* gpsData = new char[currentGPSRawData.length() + 1];
        strcpy(gpsData, currentGPSRawData.c_str());

        // Generate LoRa transmission buffer
        char* transmit_LoRa_Buffer = generate_LoRaTx_Buffer_Dynamic(gpsData, example_data_to_send_lora);
        // VERBOSE_PRINT("Before Sending");
        // VERBOSE_PRINT(transmit_LoRa_Buffer);
        // VERBOSE_PRINT(gpsData);

        delete[] gpsData;

        // Indicate the first time LoRa is transmitting
        firstTimeLoRaListening = true;
        VERBOSE_PRINT("LoRa Transmitting");

        // Send LoRa transmission
        sendLoRaChar(transmit_LoRa_Buffer, strlen(transmit_LoRa_Buffer), localAddress, destinationAddress);

        // Free the memory allocated for the LoRa transmission buffer
        delete[] transmit_LoRa_Buffer;
      } 

      // Check if it's the first time LoRa is listening
      if (firstTimeLoRaListening){
        VERBOSE_PRINT("LoRa Listening");
        firstTimeLoRaListening = false; 
      }

      // Listen for LoRa response
      receiveLoRaChar(LoRa.parsePacket(), localAddress);
    }

    delay(200);

    if (wifiEnabled && wifi_init){
        // modify the data type if needed
        // currentGPSRawDataBuffer = "$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F"; // Test Data
        // char *uartDataReceivedFromMaster = nullptr;

        char example_data_to_send_wifi[] = "Submarine Status: Depth 450 meters, Speed 18 knots, Battery Level 92%. Nearby Vessel Detected: Type - Cargo Ship, Distance - 800 meters, Bearing - 120 degrees. Weather Conditions: Moderate Sea State, Winds 15 knots from the Northeast, Water Temperature 12°C, Visibility 5 nautical miles.";
        // char example_data_to_send_wifi[] = "Call me Ishmael. Some years ago—never mind how long precisely—having little or no money in my purse, and nothing particular to interest me on shore, I thought I would sail about a little and see the watery part of the world. It is a way I have of driving off the spleen and regulating the circulation.";
        
        size_t dataSize = strlen(example_data_to_send_wifi);
        uartDataReceivedFromMaster = new char[dataSize + 1]; // Add 1 for null terminator

        currentGPSRawData = getGPSdata();
        // uartDataReceivedFromMaster = example_data_to_send_wifi;

        if (!currentGPSRawData.isEmpty()){
          // Convert String to char buffer
          currentGPSRawData.toCharArray(currentGPSRawDataBuffer, 71);

          // Pass the GPS data buffer to the function
          espnow_WiFi_send_payload(broadcastAddress, currentGPSRawDataBuffer);
        }

        if (uartDataReceivedFromMaster != nullptr){
          strcpy(uartDataReceivedFromMaster, example_data_to_send_wifi);
          espnow_WiFi_send_data(broadcastAddress, uartDataReceivedFromMaster);
          VERBOSE_PRINT("Transmitting data via WiFi: ");
          // Serial.println(uartDataReceivedFromMaster);
          // Serial.println();
        }

        unsigned long lastmillis_send = millis();
        while (sending){
          if ((millis() - lastmillis_send) >= 30000){
            finishedSending = true;
          }
          lastmillis_send = millis();
          currentGPSRawData.toCharArray(currentGPSRawDataBuffer, 71);
          espnow_WiFi_send_payload(broadcastAddress, currentGPSRawDataBuffer);
          delay(500);
        }

        unsigned long lastmillis_recv = millis();
        while (!espnow_WiFi_check_finished_receiving()){
          lastmillis_recv = millis();
          if ((millis() - lastmillis_recv) >= 30000){
            finishedReceiving = true;
          }
          if ((millis() - lastmillis_recv) >= 1000) {
            VERBOSE_PRINT("Listening via WiFi...");
          }
        }

        VERBOSE_PRINT("Transmission Received!");
        VERBOSE_PRINT(recvBuffer);
        VERBOSE_PRINT("GPS data: ");
        VERBOSE_PRINT(incomingPayload.GPS_data);

        delay(3000);

        if (recvBuffer) {
          freeRecvBuffer();
        }
        
        // delete[] uartDataReceivedFromMaster;
        // uartDataReceivedFromMaster = NULL;
    }
}

// Function to get the RSSI of the peer with the specified MAC address
void GetPeerRSSI(const uint8_t *peerMAC) {
  // PeerRSSI peerRSSI;

  bool sending1 = sending;
  sending = false;
  espnow_WiFi_send_payload(broadcastAddress, const_cast<char*>(""));
  sending = sending1;
  // Send data to peer to trigger RSSI measurement
  // esp_now_send(peerMAC, (uint8_t *) &peerRSSI, sizeof(peerRSSI));

  // Wait for response with RSSI value
  delay(1000); // Adjust delay as needed based on network conditions
}

char* generate_LoRaTx_Buffer_Dynamic(const char* gpsBuffer_Tx, const char* uartDataReceivedFromMaster) {
    // Dynamically allocate memory for the LoRa transmission buffer
    size_t max_gps_length = (gpsBuffer_Tx != nullptr) ? std::min(strlen(gpsBuffer_Tx), size_t(70)) : 0;
    size_t max_uart_length = (uartDataReceivedFromMaster != nullptr) ? std::min(strlen(uartDataReceivedFromMaster), size_t(250 - max_gps_length)) : 0;

    // Calculate the total length required for the LoRa transmission buffer
    size_t total_length = max_gps_length + max_uart_length;
    if (total_length == 0) {
        // If both inputs are empty, return nullptr
        return nullptr;
    }

    char* transmit_LoRa_Buffer = new char[total_length + 1]; // +1 for null terminator

    // Check if memory allocation was successful
    if(transmit_LoRa_Buffer == nullptr) {
        Serial.println("Error: Memory allocation failed");
        return nullptr;
    }

    // Copy GPS data to the LoRa transmission buffer if it's not empty
    if (max_gps_length > 0) {
        strncpy(transmit_LoRa_Buffer, gpsBuffer_Tx, max_gps_length);
    }

    // Null-terminate the GPS data
    transmit_LoRa_Buffer[max_gps_length] = '\0';

    // Calculate the length of GPS data copied
    size_t gps_data_length = strlen(transmit_LoRa_Buffer);

    // Copy UART data to the LoRa transmission buffer if it's not empty
    if (max_uart_length > 0) {
        strncpy(transmit_LoRa_Buffer + gps_data_length, uartDataReceivedFromMaster, max_uart_length);
    }

    // Null-terminate the LoRa transmission buffer
    transmit_LoRa_Buffer[gps_data_length + max_uart_length] = '\0';

    return transmit_LoRa_Buffer;
}

char* generate_LoRaTx_Buffer_Static(const char* gpsBuffer_Tx, const char* uartDataReceivedFromMaster) {
    // Static buffer to hold the result
    static char transmit_LoRa_Buffer[250]; // Adjusted for maximum size

    // Check if uartDataReceivedFromMaster is nullptr
    if (uartDataReceivedFromMaster == nullptr) {
        // Handle the case where uartDataReceivedFromMaster is nullptr
        // Copy the default message into transmit_LoRa_Buffer
        strcpy(transmit_LoRa_Buffer, "No UART data received");
        return transmit_LoRa_Buffer;
    }

    // Calculate the maximum length that can be copied while ensuring it doesn't exceed 250 bytes
    size_t max_gps_length = std::min(strlen(gpsBuffer_Tx), size_t(70));
    size_t max_uart_length = std::min(strlen(uartDataReceivedFromMaster), size_t(250 - max_gps_length)); // Adjusted for maximum size

    // Copy gpsBuffer_Tx to transmit_LoRa_Buffer
    strncpy(transmit_LoRa_Buffer, gpsBuffer_Tx, max_gps_length);

    // Copy uartDataReceivedFromMaster to transmit_LoRa_Buffer if there's space
    strncat(transmit_LoRa_Buffer, uartDataReceivedFromMaster, max_uart_length);

    // Null-terminate the string
    transmit_LoRa_Buffer[max_gps_length + max_uart_length] = '\0';

    return transmit_LoRa_Buffer;
}

bool Parse_LoRaRx_Buffer(const char* receive_LoRa_Buffer, char* gpsBuffer_Rx, char* LoRa_Data_For_Master) {
    // Calculate the length of transmit_LoRa_Buffer
    if (receive_LoRa_Buffer == nullptr){
      VERBOSE_PRINT("LoRa Receive Buffer Not Available");

      return false;
    }
    size_t totalLength = strlen(receive_LoRa_Buffer);
    
    // Determine the length of data to copy into gpsBuffer_Rx
    size_t gpsLength = std::min(totalLength, size_t(70));
    memcpy(gpsBuffer_Rx, receive_LoRa_Buffer, gpsLength);
    gpsBuffer_Rx[gpsLength] = '\0'; // Null-terminate the string

    // Determine the length of data to copy into LoRa_Data_For_Master
    size_t uartLength = std::min(totalLength - gpsLength, size_t(180));
    memcpy(LoRa_Data_For_Master, receive_LoRa_Buffer + gpsLength, uartLength);
    LoRa_Data_For_Master[uartLength] = '\0'; // Null-terminate the string

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
void parseGGA(String& ggaData, LatLong& latLong) {
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
