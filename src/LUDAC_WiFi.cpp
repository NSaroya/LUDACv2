#include "LUDAC_WiFi.h"

// Initialize WiFi and ESPNOW
void initLudacWIFI() {
  
  // Set device as a Wi-Fi station
  WiFi.mode(WIFI_STA);

  // Initialize packet number to zero
  out_packet_no = 0; 

  // Print the MAC address of this transceiver
  Serial.println("This device MAC address: " + s_thisAddress); 

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

// Function that gets called on every loop of the main program
void ludac_espnow_loop() {
  
  // Get readings from peripherals
  getReadings();

  // Load readings into "outgoing" payload
  strcpy(outgoing.message, out_message);
  outgoing.time = out_time;
  outgoing.packet_no = out_packet_no;

  // Send outgoing payload via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
   
  // Check for errors
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  // Print readings
  Serial.println("INCOMING READINGS");
  Serial.print("Time: ");
  Serial.print(inc_time);
  Serial.println();
  Serial.print("Incoming Message: ");
  Serial.print(inc_message);
  Serial.println();
  Serial.print("Outgoing String: ");
  Serial.print(out_message);
  Serial.println();
  Serial.print("Packet Number: ");
  Serial.print(inc_packet_no);
  Serial.println();

  // Wait before continuing loop (ms)
  delay(10000);
}


// Receive location + time from peripherals (mainly Adafruit GPS)
void getReadings() {
  s_thisAddress.toCharArray(out_message, sizeof(out_message));
  //strcpy(out_message, "test string"); //s_thisAddress;
  out_time = 500;
  out_packet_no++;
}

// Callback triggered when data is sent via ESP-NOW
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
    success = "Delivery Success";
  }
  else {
    success = "Delivery Fail";
  }
}

// Callback triggered when data is received via ESP-NOW
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  Serial.print("Bytes received: ");
  Serial.println(len);
  strcpy(inc_message, incoming.message);
  inc_time = incoming.time;
  inc_packet_no = incoming.packet_no;
}
