#ifndef LUDAC_HEADER_H
#define LUDAC_HEADER_H

#include <esp_now.h>
#include <WiFi.h>

#define LENGTH 50 // char buffer array length

// Get the MAC address of this transceiver
extern uint8_t connectedAddress[6]; // Global variable to store the connected address

// MAC Addresses
extern uint8_t broadcastAddresses[][6];

// Outgoing variables
extern char out_message[LENGTH];
extern float out_time;
extern int out_packet_no;

// Incoming variables
extern char inc_message[LENGTH];
extern float inc_time;
extern int inc_packet_no;

// Success message
extern String success;

// Define a payload struct
typedef struct payload {
    char message[LENGTH];
    float time;
    int packet_no;
} payload;

// Payloads
extern payload outgoing;
extern payload incoming;
extern esp_now_peer_info_t peerInfo;

// Function prototypes
bool initLudacWIFI();
bool WiFi_connectToPeer(uint8_t* address);
bool WiFi_RegisterPeer();
void getReadingsFromAddress();
void getReadingsFromArray();
void espnow_WiFi_duplex();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
int32_t getWiFiRSSI();

#endif // LUDAC_HEADER_H
