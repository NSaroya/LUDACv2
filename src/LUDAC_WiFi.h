#ifndef LUDAC_HEADER_H
#define LUDAC_HEADER_H

#include <esp_now.h>
#include <WiFi.h>

#define LENGTH 50 // char buffer array length

// Get the MAC address of this transceiver
String s_thisAddress = WiFi.macAddress(); 

// MAC Addresses
uint8_t broadcastAddresses[][6] = {
  {0x40, 0x44, 0xD8, 0x08, 0xF9, 0x94}, // MAC address of transceiver A (HackED sticker)
  {0x40, 0x22, 0xD8, 0x06, 0x75, 0x2C}, // MAC address of transceiver B (no sticker)
  {0xB8, 0xD6, 0x1A, 0x67, 0xF8, 0x54}, // MAC address of transceiver C (covered microstrip antenna)
  {0xA0, 0xA3, 0xB3, 0x89, 0x23, 0xE4}  // MAC address of transceiver D (weird WOER antenna, doesn't receive)
};

// MAC ADDRESSES
extern String s_thisAddress;
extern uint8_t broadcastAddress[];

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
void initLudacWIFI();
bool WiFi_connectToPeer(uint8_t* address);
void WiFi_RegisterPeer();
void getReadingsFromAddress();
void getReadingsFromArray();
void espnow_WiFi_duplex();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

#endif // LUDAC_HEADER_H
