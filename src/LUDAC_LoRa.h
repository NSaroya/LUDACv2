#ifndef LUDAC_LORA_H
#define LUDAC_LORA_H

#include <Arduino.h>
#include "LoRa.h"

// Define your constants here
#define RADIO_CS_PIN 5
#define RADIO_DI0_PIN 2
#define RADIO_RST_PIN 15
#define LORA_FREQUENCY 915E6
#define LORA_SPREADING_FACTOR 8

// Function prototypes
bool initLoRa();
void sendLoRaChar(char outgoing[], int buffer_size, byte localAddress, byte destinationAddress);
void receiveLoRaChar(int packetSize, int buffer_size, byte localAddress, char* incoming);
void recParsing(char received[]);
void sendLoRaString(String outgoing, byte localAddress, byte destinationAddress);
bool receiveLoRaString(int packetSize, byte localAddress, String &incoming);

// Global variables
extern char LoRa_sending_buffer[50];
extern char LoRa_received_buffer[50];
extern char lat_rec[15];
extern char lon_rec[15];
extern char dis_rec[15];
extern float lat_away;
extern float lon_away;
extern float dis_away;

#endif // LUDAC_LORA_H
