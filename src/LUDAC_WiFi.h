#ifndef LUDAC_HEADER_H
#define LUDAC_HEADER_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Get the MAC address of this transceiver
String s_thisAddress = WiFi.macAddress(); 

/**
"40:44:D8:08:F9:94"; // MAC address of transceiver A (HackED sticker)
"40:22:D8:06:75:2C"; // MAC address of transceiver B (no sticker)
"B8:D6:1A:67:F8:54"; // MAC address of transceiver C (covered microstrip antenna)
"A0:A3:B3:89:23:E4"; // MAC address of transceiver D (weird WOER antenna, doesn't receive)
*/

// MAC ADDRESSES (uncomment the address of the device that is receiving)
//uint8_t broadcastAddress[] = {0x40, 0x44, 0xD8, 0x08, 0xF9, 0x94}; // MAC address of transceiver A (HackED sticker)
//uint8_t broadcastAddress[] = {0x40, 0x22, 0xD8, 0x06, 0x75, 0x2C}; // MAC address of transceiver B (no sticker)
uint8_t broadcastAddress[] = {0xB8, 0xD6, 0x1A, 0x67, 0xF8, 0x54}; // MAC address of transceiver C (covered microstrip antenna)
//uint8_t broadcastAddress[] = {0xA0, 0xA3, 0xB3, 0x89, 0x23, 0xE4}; // MAC address of transceiver D (weird WOER antenna, doesn't receive)

#define LENGTH 50 // char buffer array length

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
void ludac_espnow_loop();
void getReadings();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

#endif // LUDAC_HEADER_H
