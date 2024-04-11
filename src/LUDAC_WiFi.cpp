/**
 * @file LUDAC_WiFi.cpp
 * @brief ESP-NOW Communication Module.
 * 
 * This module provides functionality to establish communication
 * between ESP32 devices using ESP-NOW protocol.
 */

#include "LUDAC_WiFi.h"

// Define global variables
uint8_t connectedAddress[6] = {0}; // Initialize to all zeros

uint8_t broadcastAddresses[][6] = {
  {0x40, 0x44, 0xD8, 0x08, 0xF9, 0x94}, // MAC address of transceiver A (HackED sticker)
  {0x40, 0x22, 0xD8, 0x06, 0x75, 0x2C}, // MAC address of transceiver B (no sticker)
  {0xB8, 0xD6, 0x1A, 0x67, 0xF8, 0x54}, // MAC address of transceiver C (covered microstrip antenna)
  {0xA0, 0xA3, 0xB3, 0x89, 0x23, 0xE4}  // MAC address of transceiver D (weird WOER antenna, doesn't receive)
};

String s_thisAddress = WiFi.macAddress(); 

char out_message[LENGTH];
float out_time;
int out_packet_no;

char inc_message[LENGTH];
float inc_time;
int inc_packet_no;

String success;

payload outgoing;
payload incoming;
esp_now_peer_info_t peerInfo;

typedef struct {
  unsigned frame_ctrl: 16;
  unsigned duration_id: 16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl: 16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

int rssi_display;

/**
 * @brief Initializes WiFi in station mode.
 * 
 * This function initializes WiFi in station mode and waits until 
 * connected to a WiFi network.
 */
bool initLudacWIFI() {

  // Disconnect from the WiFi network and disable WiFi
  WiFi.disconnect(true); 

  // Set device as a Wi-Fi station
  WiFi.mode(WIFI_STA);

  // Returns the number of networks found
  // WiFi.scanNetworks();

  // byte count = 0;
  // while (WiFi.status() != WL_CONNECTED && count < 15) {
  //   count++;
  //   delay(500);
  //   Serial.println("Connecting ...");
  // }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting...OK.");
  } else {
    Serial.println("Connecting...Failed.");
  }

  Serial.println("WIFI Setup done");

  delay(500);

  return true;
}

/**
 * @brief Connects to peers using the provided addresses.
 * 
 * This function attempts to add each broadcast address as a peer
 * and returns true if any peer is added successfully.
 * 
 * @param addresses Array of broadcast addresses.
 * @param numAddresses Number of addresses in the array.
 * @return true if any peer is added successfully, otherwise false.
 */
bool WiFi_connectToPeer(uint8_t (*addresses)[6], size_t numAddresses) {
  uint8_t* broadcastAddress;

  // Iterate over each broadcast address
  for (size_t i = 0; i < sizeof(broadcastAddresses) / sizeof(broadcastAddresses[0]); ++i) {
    // Copy the current broadcast address
    broadcastAddress = broadcastAddresses[i];

    // Copy the broadcast address to peerInfo
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Attempt to add the peer
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
      Serial.println("Peer added successfully");
      memcpy(connectedAddress, broadcastAddress, 6);
      return true; // Return true if peer added successfully
    }
  }

  // If none of the broadcast addresses could be added as a peer
  Serial.println("Failed to add any peer");
  return false;
}

/**
 * @brief Connects to a peer manually using the provided address.
 * 
 * This function attempts to add a peer with the provided address
 * and returns true if successful.
 * 
 * @param address The address of the peer to connect to.
 * @return true if the peer is added successfully, otherwise false.
 */
bool WiFi_connectToPeer_manual(uint8_t* address){
  memcpy(peerInfo.peer_addr, address, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return false;
  }
  
  return true;
}

/**
 * @brief Registers the device as a peer and initializes ESP-NOW.
 * 
 * This function registers the device as a peer, initializes ESP-NOW,
 * and sets up callbacks for data transmission and reception.
 */
bool WiFi_RegisterPeerAuto() {
  // Initialize packet number to zero
  out_packet_no = 0; 

  // Print the MAC address of this transceiver
  Serial.println("This device MAC address: " + s_thisAddress); 

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return false;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  bool WifiConnectStatus = WiFi_connectToPeer(broadcastAddresses, 
                            sizeof(broadcastAddresses) / sizeof(broadcastAddresses[0]));

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  return WifiConnectStatus;
}

/**
 * @brief Registers the device as a peer and initializes ESP-NOW.
 * 
 * This function registers the device as a peer, initializes ESP-NOW,
 * and sets up callbacks for data transmission and reception.
 */
bool WiFi_RegisterPeerManual(uint8_t* broadcastAddress) {
  // Initialize packet number to zero
  out_packet_no = 0; 

  // Print the MAC address of this transceiver
  Serial.println("This device MAC address: " + s_thisAddress); 

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return false;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(sendWiFiCallback);

  // Register peer
  bool WifiConnectStatus = WiFi_connectToPeer_manual(broadcastAddress);

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(recvWiFiCallback);

  // esp_wifi_set_promiscuous(true);
  // esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);

  return WifiConnectStatus;
}

void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  // All espnow traffic uses action frames which are a subtype of the mgmnt frames so filter out everything else.
  if (type != WIFI_PKT_MGMT)
    return;

  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
  const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
  const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

  int rssi = ppkt->rx_ctrl.rssi;
  rssi_display = rssi;
}

/**
 * @brief Gets readings from the device address.
 * 
 * This function copies the device address to the outgoing message.
 */
void getReadingsFromAddress() {
  strcpy(out_message, s_thisAddress.c_str());
  out_time = millis();
  out_packet_no++;
}

/**
 * @brief Gets readings from the device address as an array.
 * 
 * This function converts the device address to a character array
 * and stores it in the outgoing message.
 */
void getReadingsFromArray() {
  s_thisAddress.toCharArray(out_message, sizeof(out_message));
  out_time = 500;
  out_packet_no++;
}

/**
 * @brief Sends data via ESP-NOW.
 * 
 * This function sends outgoing payload via ESP-NOW and prints status.
 */
void espnow_WiFi_duplex() {
  
  // WiFi.RSSI();

  // Get readings from peripherals
  getReadingsFromArray();

  // Load readings into "outgoing" payload
  strcpy(outgoing.message, out_message);
  outgoing.time = out_time;
  outgoing.packet_no = out_packet_no;

  // Send outgoing payload via ESP-NOW
  esp_err_t result = esp_now_send(connectedAddress, (uint8_t *) &outgoing, sizeof(outgoing));
   
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

/**
 * @brief Callback triggered when data is sent via ESP-NOW.
 * 
 * This function prints the status of the last packet send operation.
 * 
 * @param mac_addr MAC address of the recipient.
 * @param status Status of the send operation.
 */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  success = (status == ESP_NOW_SEND_SUCCESS) ? "Delivery Success" : "Delivery Fail";
}

/**
 * @brief Callback triggered when data is received via ESP-NOW.
 * 
 * This function handles received data and prints the received bytes.
 * 
 * @param mac MAC address of the sender.
 * @param incomingData Pointer to the received data.
 * @param len Length of the received data.
 */
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  strcpy(inc_message, incoming.message);
  inc_time = incoming.time;
  inc_packet_no = incoming.packet_no;
}

/**
 * @brief Get the received signal strength indicator (RSSI) from WiFi.
 * 
 * This function retrieves the RSSI value from the WiFi module. If the RSSI
 * value is not equal to 0, it returns the RSSI value; otherwise, it 
 * returns -1 to indicate an invalid RSSI value.
 * 
 * @return The RSSI value if it is valid, otherwise -1.
 */
int8_t getWiFiRSSI() {
  int8_t RSSI = WiFi.RSSI();
  return (RSSI != 0) ? RSSI : -1; // Return RSSI value if not equal to 0, else return -1
}

