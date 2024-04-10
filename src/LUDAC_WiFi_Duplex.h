#ifndef PACKET_HANDLING_H
#define PACKET_HANDLING_H

#include <Arduino.h>
#include <esp_now.h>
#include <LUDAC_WiFi.h>
#include <WiFi.h>


#define MAX_DATA_SIZE 1000 // Max size for raw data stored on device (1KB for now)
#define PACKET_SIZE 10 // Packet size in bytes
#define GPS_DATA_SIZE 71 // GPS data size in bytes


// --TYPEDEF STRUCTS--
// Packet struct
typedef struct Packet {
    char packet_data[PACKET_SIZE]; // Packet of data from to be sent/received
    uint16_t packet_number; // Increments on successful transmission
    size_t packet_size; // Actual packet size; not PACKET_SIZE
    size_t data_size; // Cumulative size of the total transmission 
} Packet;

// Payload struct (note: can be renamed from payload2 to payload)
typedef struct payload2 {
    char GPS_data[GPS_DATA_SIZE]; // Placeholder for GPS data, edit as needed
    Packet packet;
    bool first_packet; // TRUE when the packet is the first packet
    bool final_packet; // TRUE when the packet is the final packet
    int payload_number; // Increments on every transmission, successful or not 
} payload2;


// --GLOBAL VARIABLES--
// Flags
extern bool sending; // Flag for if data is being sent
extern bool receiving; // Flag for if data is being received
extern bool finishedSending; // Flag set after data is finished sending
extern bool finishedReceiving; // Flag set after data is finished being received

extern bool test_flag;

// Counters and indices
extern size_t recvIndex; // Points to the next unwritten byte in recvBuffer
extern size_t sendIndex; // Points to the next unsent byte in sendBuffer
extern size_t sendBufferSize; // Actual size of data to be sent; not MAX_DATA_SIZE
extern size_t recvBufferSize; // Actual size of received data; not MAX_DATA_SIZE

// Buffers and packets
extern char* sendBuffer; // Data to be transmitted, dynamically allocated
extern char* recvBuffer; // Holds received data

extern payload2 outgoingPayload; // Immediate next payload to be sent
extern payload2 incomingPayload; // Payload just received

    extern Packet outgoingPacket; // Immediate next packet to be sent
    extern Packet incomingPacket; // Packet just received
    extern Packet emptyPacket; // Placeholder that contains nothing

    extern uint16_t send_packet_number; // Increments on successful transmission
    extern size_t send_packet_size; // Size of packet to be sent

// Example data to send
extern char placeholder_GPS_data[];

// -FUNCTION PROTOTYPES-

// Main program initialization functions:
extern void WiFi_Packet_Handling_init(); // Call in main initialization routine
extern void sendWiFiCallback(const uint8_t * mac_addr, esp_now_send_status_t status); // Register with esp_now_register_recv_cb(sendWiFiCallback)
extern void recvWiFiCallback(const uint8_t * mac, const uint8_t * incomingData, int len); // Register with esp_now_register_send_cb(recvWiFiCallback)

// Main program loop functions:
// extern void espnow_WiFi_send_payload(); // Call in main program loop
void espnow_WiFi_send_payload(uint8_t* broadcastAddress, char* placeholder_GPS_data);

extern bool espnow_WiFi_check_finished_receiving(); // Call in main program loop

// extern void espnow_WiFi_send_data(char * data_to_send); // Call to start sending new data
bool espnow_WiFi_send_data(uint8_t* broadcastAddress, char* data_to_send);

// Functions that are not neccessary to call in the main program:
extern void espnow_WiFi_recv_payload();
extern void freeSendBuffer();
extern void freeRecvBuffer();

#endif