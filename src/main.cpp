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

// Define verbose printing macro for debugging
#define VERBOSE_PRINT(x) \
    Serial.print(millis()); \
    Serial.print(": "); \
    Serial.println(x);

// Function prototypes
void taskGPS(void *pvParameters);
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

int WiFi_RxData_Flush_Wait = 5000;  // Wait for 5 seconds, then free the received data
uint32_t last_WiFi_RxData_Flush_Time = 0;

unsigned long lastRSSICheckTime = 0;
const unsigned long checkRSSIInterval = 10000; // 10 seconds

int WiFi_RSSI = 0;
int LoRa_RSSI = 0;
bool wifiEnabled, loraEnabled = false;

char gpsBuffer_Tx[70] = {}; // 70 characters fixed
char gpsBuffer_Rx[70] = {}; // 70 characters fixed
char LoRa_Data_For_Master[180] = {};

String currentGPSRawData = "";
char currentGPSRawDataBuffer[71] = {}; // Buffer to hold GPS data (including null terminator)
String dataFromDestinationAddress = "";

uint8_t broadcastAddress[] = {0xB8, 0xD6, 0x1A, 0x67, 0xF8, 0x54}; 
  // {0x40, 0x44, 0xD8, 0x08, 0xF9, 0x94}, // MAC address of transceiver A (HackED sticker)
  // {0x40, 0x22, 0xD8, 0x06, 0x75, 0x2C}, // MAC address of transceiver B (no sticker)
  // {0xB8, 0xD6, 0x1A, 0x67, 0xF8, 0x54}, // MAC address of transceiver C (covered microstrip antenna)
  // {0xA0, 0xA3, 0xB3, 0x89, 0x23, 0xE4}  // MAC address of transceiver D (weird WOER antenna, doesn't receive)

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
    if (!initLudacLoRa()) {
        VERBOSE_PRINT("Starting LoRa failed!");
    } else {
        VERBOSE_PRINT("Starting LoRa succeeded!");
    }

    // Initialize Wi-Fi module or register with a peer device
    if (initLudacWIFI()) {
        VERBOSE_PRINT("Starting WiFI in Progress...");
        WiFi_RegisterPeerManual(broadcastAddress);
        WiFi_Packet_Handling_init();
        VERBOSE_PRINT("Starting WiFI succeeded!");
    } else {
        VERBOSE_PRINT("Starting WiFI failed!");
    }

    // Initialize GPS module
    VERBOSE_PRINT("Initializing GPS ...");
    
    if (EnableDualCoreforGPS){
      // Create a task for GPS processing
      xTaskCreatePinnedToCore(taskGPS, "taskGPS", 4096, NULL, 1, NULL, 1); // Core 1
    } else {
      initLudacGPS();
    }

    VERBOSE_PRINT("ok1");

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

void loop(){

    // String printGPSI = getGPSdata();
    VERBOSE_PRINT("Chilling ...");
    delay(1000);
}

void taskGPS(void *pvParameters) {
    (void) pvParameters;

    // Initialize GPS module
    initLudacGPS();

    for (;;) {
        // if (receivedGPSfix()) {
        String gpsData = getGPSdata();
        VERBOSE_PRINT("GPS Data: " + gpsData);
        // }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}

void loop1() {
    
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
        
        if (loraEnabled){
          // Send gpsBuffer_Rx
          for (size_t i = 0; i < strlen(gpsBuffer_Rx); ++i) {
              Serial_UART_RaspPi.write(gpsBuffer_Rx[i]);
          }

          // Send LoRa_Data_For_Master
          for (size_t i = 0; i < strlen(LoRa_Data_For_Master); ++i) {
              Serial_UART_RaspPi.write(LoRa_Data_For_Master[i]);
          }
        }

        if (wifiEnabled){
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

    // Get the current time
    unsigned long currentTime = millis();

    // Check if it's time to update RSSI values
    if (currentTime - lastRSSICheckTime > checkRSSIInterval && (lastRSSICheckTime = currentTime, true)) {
        // Update Wi-Fi RSSI if available
        int wifiRSSI = getWiFiRSSI();
        if (wifiRSSI != -1) WiFi_RSSI = wifiRSSI;

        // Update LoRa RSSI if available
        int loraRSSI = getLoRaRSSI();
        if (loraRSSI != -1) LoRa_RSSI = loraRSSI;
    }

    // Determine if Wi-Fi is enabled based on RSSI values
    wifiEnabled = (WiFi_RSSI != -1 && (LoRa_RSSI == -1 || WiFi_RSSI > LoRa_RSSI));

    // Determine if LoRa is enabled based on RSSI values
    loraEnabled = (LoRa_RSSI != -1 && (WiFi_RSSI == -1 || LoRa_RSSI > WiFi_RSSI));

    // If neither Wi-Fi nor LoRa has a valid RSSI value, keep both off
    if (WiFi_RSSI == -1 && LoRa_RSSI == -1) {
        wifiEnabled = false;
        loraEnabled = false;
    }

    if (currentTime - lastTxSendTime > sendTxInterval) {

      if (loraEnabled){

        // Call generate_LoRaTx_Buffer to create transmit_LoRa_Buffer
        char* transmit_LoRa_Buffer = generate_LoRaTx_Buffer(gpsBuffer_Tx, uartDataReceivedFromMaster);

        delete[] uartDataReceivedFromMaster;
        uartDataReceivedFromMaster = nullptr;
        
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
          // modify the data type if needed
          // currentGPSRawDataBuffer = "$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F"; // Test Data
          // char *uartDataReceivedFromMaster = nullptr;
          // uartDataReceivedFromMaster = modify needed ... 

          if (!currentGPSRawData.isEmpty()){
            // Convert String to char buffer
            currentGPSRawData.toCharArray(currentGPSRawDataBuffer, 71);

            // Pass the GPS data buffer to the function
            espnow_WiFi_send_payload(broadcastAddress, currentGPSRawDataBuffer);
          }

          if (uartDataReceivedFromMaster != nullptr){
            espnow_WiFi_send_data(broadcastAddress, uartDataReceivedFromMaster);
          }

          while (sending){
            currentGPSRawData.toCharArray(currentGPSRawDataBuffer, 71);
            espnow_WiFi_send_payload(broadcastAddress, currentGPSRawDataBuffer);
          }

          while (!espnow_WiFi_check_finished_receiving()){
            Serial.print("Transmission Receiving: ");
          }

          Serial.print("Transmission Received: ");
          Serial.println(recvBuffer);
          Serial.println("GPS data: ");
          Serial.println(incomingPayload.GPS_data);

          if (millis() - last_WiFi_RxData_Flush_Time > WiFi_RxData_Flush_Wait){
            int last_WiFi_RxData_Flush_Time = millis();
            freeRecvBuffer();
          }
          
          delete[] uartDataReceivedFromMaster;
          uartDataReceivedFromMaster = nullptr;
      }
      
    for (int tryCount = 0; tryCount < 10 && !currentLocation.hasValidFix; tryCount++){
    // if (1){

      // Reset struct to default
      currentLocation.reset();
      // Reset buffer to all zeros
      memset(gpsBuffer_Tx, 0, sizeof(gpsBuffer_Tx));

      currentGPSRawData = getGPSdata();

      VERBOSE_PRINT("check1");
      VERBOSE_PRINT(currentGPSRawData);

      currentGPSRawData.toCharArray(gpsBuffer_Tx, sizeof(gpsBuffer_Tx));
      
      VERBOSE_PRINT("check2");
      VERBOSE_PRINT(currentGPSRawData);

      // String ggaData = "$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F"; // TEST DATA
      parseGGA(currentGPSRawData, currentLocation);
      printGPSInfo();

      delay(500);
    }
  }
}

char* generate_LoRaTx_Buffer(const char* gpsBuffer_Tx, const char* uartDataReceivedFromMaster) {
    // Check if uartDataReceivedFromMaster is nullptr
    if (uartDataReceivedFromMaster == nullptr) {
        // Handle the case where uartDataReceivedFromMaster is nullptr
        // Here, I'm returning a default buffer or you can choose to handle it differently based on your requirements.
        static char defaultBuffer[251] = "No UART data received";
        // Fill the rest of the buffer with null characters
        memset(defaultBuffer + strlen(defaultBuffer), '\0', sizeof(defaultBuffer) - strlen(defaultBuffer));
        return defaultBuffer;
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
