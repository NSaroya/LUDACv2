
#include "LUDAC_LoRa.h"
// #include "../docs/LoRa.h"

/**
 * @brief Initialize the LoRa module.
 * 
 * This function sets up the LoRa module with the specified pins and frequency.
 * 
 * @return True if initialization is successful, false otherwise.
 */
bool initLoRa() {

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
 * @param buffer_size The size of the character array.
 * @param localAddress The local address of the receiver.
 * @param incoming A pointer to a character array to store the incoming message.
 */
void receiveLoRaChar(int packetSize, int buffer_size, byte localAddress, char* incoming) {
  
  // Check if there is no data received
  if (packetSize == 0)
    return;

  // Read recipient, sender, message ID, and message length from LoRa packet
  int recipient = LoRa.read();
  byte sender = LoRa.read();
  byte incomingMsgId = LoRa.read();
  byte incomingLength = LoRa.read();

  // Initialize a buffer to store the received message
  char incomingBuffer[buffer_size] = {};

  // Loop to read each character from LoRa and store it in the buffer
  for (int m = 0; m < buffer_size; ++m) {
    incomingBuffer[m] += LoRa.read();
  }
  
  // Check if the received message length matches the expected length
  if (incomingLength != buffer_size) {
    Serial.println("error, message length does not match");
    return;
  }

  // Check if the message is intended for this device
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me");
    return;
  }

  // Print information about the received message
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length " + String(incomingLength));

  // Print the received message
  for(int i = 0; i < buffer_size; ++i) {
    Serial.print(incomingBuffer[i]);
  }
  Serial.println("; RSSI" + String(LoRa.packetRssi()));
  Serial.println();

  // Copy the received message to the provided buffer
  for(int j = 0; j < buffer_size; ++j) {
    incoming[j] = incomingBuffer[j];
  }

  // Parse the received message
  recParsing(incoming);

  // Print parsed data
  Serial.println(lat_away);
  Serial.println(lon_away);
  Serial.println(dis_away);
}


void recParsing(char received[]){
  
  // Parse the received char array by store the corresponding parts to their contaiers
  for(int i = 0; i<15; i++){
    lat_rec[i] = received[i];
    lat_away = atof(lat_rec);
  }

  for(int j = 0; j<15; j++){
    lon_rec[j] = received[j+15];
    lon_away = atof(lon_rec);
  }

  for(int k = 0; k<15; k++){
    dis_rec[k] = received[k+30];
    dis_away = atof(dis_rec);
  }
}

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

