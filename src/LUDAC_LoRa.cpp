/**
 * @file LUDAC_LoRa.cpp
 * @brief LoRa Communication Module.
 * 
 * This module provides functionality to establish communication
 * between ESP32 devices using LoRa protocol.
 */

#include "LUDAC_LoRa.h"

const int MAX_LORA_BUFFER_SIZE = 250;
char LoRa_received_buffer[250];

// byte msgCount = 0;
// long lastSendTime = 0;
// int interval = 2000;

/**
 * @brief Initialize the LoRa module.
 * 
 * This function sets up the LoRa module with the specified pins and frequency.
 * 
 * @return True if initialization is successful, false otherwise.
 */
bool initLudacLoRa() {

  // Initiate the received buffer and the lon, lat, distance containers for the received data
  char lat_rec[15] = {0};
  char lon_rec[15] = {0};
  char dis_rec[15] = {0};
  float lat_away;
  float lon_away;
  float dis_away;

  // Set up LoRa module
  LoRa.setPins(RADIO_CS_PIN, RADIO_DI0_PIN, RADIO_RST_PIN);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    return false;
  }

  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  return true;
}

/**
 * @brief Send data via LoRa.
 * 
 * This function sends data over LoRa communication.
 * 
 * @param outgoing The character array to be sent.
 * @param buffer_size The size of the character array.
 * @param localAddress The local address of the sender.
 * @param destinationAddress The address of the recipient.
 */
void sendLoRaChar(char outgoing[], int buffer_size, byte localAddress, byte destinationAddress) {
  
  // Begin LoRa packet transmission
  LoRa.beginPacket();

  // Write destination address, local address, and buffer size to LoRa packet
  LoRa.write(destinationAddress);
  LoRa.write(localAddress);
  LoRa.write(buffer_size);

  // Iterate through the character array to send each character over LoRa
  for (int n = 0; n < buffer_size; ++n) {
    LoRa.print(outgoing[n]); // Send character over LoRa
    Serial.print(outgoing[n]); // Print character to Serial for debugging
  }
    
  // End LoRa packet transmission
  LoRa.endPacket();
}


/**
 * @brief Receive data via LoRa.
 * 
 * This function receives data over LoRa communication.
 * 
 * @param packetSize The size of the received packet.
 * @param localAddress The local address of the receiver.
 * @return char* A pointer to the received buffer, or nullptr if there was an error.
 */
char* receiveLoRaChar(int packetSize, byte localAddress) {
  
  // Check if there is no data received
  if (packetSize == 0)
    return nullptr;

  // Read recipient, sender, message ID, and message length from LoRa packet
  int recipient = LoRa.read();
  byte sender = LoRa.read();
  byte incomingMsgId = LoRa.read();
  byte incomingLength = LoRa.read();

  // Loop to read each character from LoRa and store it in the buffer
  for (int m = 0; m < MAX_LORA_BUFFER_SIZE; ++m) {
    LoRa_received_buffer[m] = LoRa.read();
  }
  
  // Check if the received message length matches the expected length
  if (incomingLength != MAX_LORA_BUFFER_SIZE) {
      Serial.println("LoRa: Error, message length does not match");
  }

  // Check if the message is intended for this device
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("LoRa: This message is not for me");
    return nullptr;
  }

  // Print information about the received message
  Serial.println("LoRa: Received from: 0x" + String(sender, HEX));
  Serial.println("LoRa: Sent to 0x" + String(recipient, HEX));
  Serial.println("LoRa: Message ID: " + String(incomingMsgId));
  Serial.println("LoRa: Message length " + String(incomingLength));

  return LoRa_received_buffer;
}

/**
 * @brief Parse received character data
 * 
 * This function parses data received over LoRa communication.
 * 
 * @param received The received character array.
 */
void receiveLoRaChar_Parse(char received[]){
  
  // Parse the received char array by storing the corresponding parts in their containers
  for(int i = 0; i<15; i++){
    lat_rec[i] = received[i];
    lat_away = atof(lat_rec);  // Convert latitude string to double
  }

  for(int j = 0; j<15; j++){
    lon_rec[j] = received[j+15];
    lon_away = atof(lon_rec);  // Convert longitude string to double
  }

  for(int k = 0; k<15; k++){
    dis_rec[k] = received[k+30];
    dis_away = atof(dis_rec);  // Convert distance string to double
  }
}


// float relaDistance(float lat1, float lon1, float lat2, float lon2) {
  
//   // Pre-calculate some parameters to speed up the algorithm
//   float delta_lat = (lat2-lat1)*PI/180;
//   float delta_lon = (lon2-lon1)*PI/180;
//   float sin_delta_lat = sin(delta_lat/2);
//   float sin_delta_lon = sin(delta_lon/2);

//   // Implement the GPS distance formula
//   float d = 2*6378000*asin(sqrt(sin_delta_lat*sin_delta_lat+cos(lat1*PI/180)*cos(lat2*PI/180)*sin_delta_lon*sin_delta_lon));
//   return d;
// }

/**
 * @brief Send data via LoRa.
 * 
 * This function sends data over LoRa communication.
 * 
 * @param outgoing The string message to be sent.
 * @param localAddress The local address of the sender.
 * @param destinationAddress The address of the recipient.
 */
void sendLoRaString(String outgoing, byte localAddress, byte destinationAddress) {

  // Begin LoRa packet transmission
  LoRa.beginPacket();

  // Write destination address, local address, and buffer size to LoRa packet
  LoRa.write(destinationAddress);
  LoRa.write(localAddress);
  LoRa.write(outgoing.length());
  LoRa.print(outgoing);

  // End LoRa packet transmission
  LoRa.endPacket();
}

/**
 * @brief Receive a string message via LoRa.
 * 
 * This function receives a string message over LoRa communication.
 * 
 * @param packetSize The size of the received packet.
 * @param localAddress The local address of the receiver.
 * @param incoming A reference to a string to store the incoming message.
 * @return True if the message is received successfully, false otherwise.
 */
bool receiveLoRaString(int packetSize, byte localAddress, String &incoming) {
  
  // Check if there's no incoming data
  if (packetSize == 0) {
    return false;
  }

  // Read recipient, sender, and message length
  int recipient = LoRa.read();
  byte sender = LoRa.read();
  byte incomingLength = LoRa.read();

  // Clear the incoming string
  incoming = "";

  // Read and store the incoming message
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  // Check if the received message length matches the expected length
  if (incomingLength != incoming.length()) {
    // Log error: Message length does not match
    return false;
  }

  // Check if the recipient address matches the local address
  if (recipient != localAddress) {
    // Log error: Recipient address does not match local address
    return false;
  }

  // Message received successfully
  return true;
}

/**
 * @brief Get the received signal strength indicator (RSSI) from LoRa.
 * 
 * This function retrieves the RSSI value from the LoRa module. If the RSSI
 * value is not equal to -157, it returns the RSSI value; otherwise, it 
 * returns -1 to indicate an invalid RSSI value.
 * 
 * @return The RSSI value if it is valid, otherwise -1.
 */
int getLoRaRSSI(){
  int rssi = LoRa.packetRssi();
  return (rssi != -157 && rssi != -164) ? rssi : -1; // Return -1 if RSSI is -157 or -164
}

/**
 * @brief Get the signal-to-noise ratio (SNR) from LoRa.
 * 
 * This function retrieves the SNR value from the LoRa module. If the SNR
 * value is not equal to 0, it returns the SNR value; otherwise, it 
 * returns -1 to indicate an invalid SNR value.
 * 
 * @return The SNR value if it is valid, otherwise -1.
 */
float getLoRaSNR(){
  float snr = LoRa.packetSnr();
  return (snr != 0) ? snr : -1; // Return SNR value if not equal to 0, else return -1
}
