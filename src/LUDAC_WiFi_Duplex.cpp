/* Author: Ethan Wilson
 * 
 * ---=== INSTRUCTIONS FOR HOW TO INTEGRATE THIS CODE INTO THE MAIN PROGRAM ===---
 * 
 * IN THE MAIN INITIALIZATION ROUTINE:
 *  - Add the following lines:
 *      o esp_now_register_recv_cb(recvWiFiCallback);
 *      o esp_now_register_send_cb(sendWiFiCallback);
 *      o packet_handling_init();
 * 
 * IN THE MAIN PROGRAM LOOP:
 *  - Add the following lines:
 *      o espnow_WiFi_send_payload();
 *      o espnow_WiFi_check_finished_receiving();
 *  - To start sending data:
 *      o Call 'espnow_WiFi_send_data(pointer_to_char_array);'
 * 
 * IN THIS FILE:
 *  - Edit 'espnow_WiFi_send_payload()' to handle sending GPS data.
 *  - Edit 'espnow_WiFi_recv_payload()' to handle receiving GPS data.
 *  - Ensure necessary includes are accounted for.
 * 
 */

#include <LUDAC_WiFi.h>

// --GLOBAL VARIABLES--
// Flags
bool sending; // Flag for if data is being sent
bool receiving; // Flag for if data is being received
bool finishedSending; // Flag set after data is finished sending
bool finishedReceiving; // Flag set after data is finished being received

bool test_flag;

// Counters and indices
size_t recvIndex; // Points to the next unwritten byte in recvBuffer
size_t sendIndex; // Points to the next unsent byte in sendBuffer
size_t sendBufferSize; // Actual size of data to be sent; not MAX_DATA_SIZE
size_t recvBufferSize; // Actual size of received data; not MAX_DATA_SIZE

// Buffers and packets
char* sendBuffer = NULL; // Data to be transmitted, dynamically allocated
char* recvBuffer = NULL; // Holds received data

payload2 outgoingPayload; // Immediate next payload to be sent
payload2 incomingPayload; // Payload just received

    Packet outgoingPacket; // Immediate next packet to be sent
    Packet incomingPacket; // Packet just received
    Packet emptyPacket; // Placeholder that contains nothing

    uint16_t send_packet_number; // Increments on successful transmission
    size_t send_packet_size; // Size of packet to be sent

// Example data to send
char placeholder_GPS_data[] = "Placeholder GPS data.";

// -FUNCTION PROTOTYPES-

// Main program initialization functions:
void packet_handling_init(); // Call in main initialization routine
void sendWiFiCallback(const uint8_t * mac_addr, esp_now_send_status_t status); // Register with esp_now_register_recv_cb(sendWiFiCallback)
void recvWiFiCallback(const uint8_t * mac, const uint8_t * incomingData, int len); // Register with esp_now_register_send_cb(recvWiFiCallback)

// Main program loop functions:
void espnow_WiFi_send_payload(); // Call in main program loop
bool espnow_WiFi_check_finished_receiving(); // Call in main program loop
void espnow_WiFi_send_data(char * data_to_send); // Call to start sending new data

// Functions that are not neccessary to call in the main program:
void espnow_WiFi_recv_payload();
void freeSendBuffer();
void freeRecvBuffer();

// --FUNCTION DEFINTIONS--

void espnow_WiFi_send_payload(uint8_t* broadcastAddress, char* placeholder_GPS_data) {

    /*
     *
     * [GPS DATA MUST BE LOADED INTO THE PAYLOAD HERE]
     *
     */

    strcpy(outgoingPayload.GPS_data, placeholder_GPS_data);
    
    
    // Increment payload number
    outgoingPayload.payload_number++;

    // Gets the next packet from the send buffer and prepares it to send.
    // If sending = false, then it sends an empty packet
        // finishedSending is set when the final packet is successfully received
    if (finishedSending)  {
        send_packet_number = 0;
        outgoingPacket.packet_number = 0;
        sendIndex = 0;
        sending = false;
        outgoingPayload.final_packet = false;
        freeSendBuffer();
        finishedSending = false;
    }
    
    if (sending) {

        // Send cumulative data size
        if (sendBuffer) {
            
            outgoingPacket.data_size = sendBufferSize;
            
            // Calculate packet size
            outgoingPacket.packet_size = (sendBufferSize - sendIndex < PACKET_SIZE) ? (sendBufferSize - sendIndex) : PACKET_SIZE;

            // Load the next packet of data from sendBuffer into outgoingPacket
            memcpy(outgoingPacket.packet_data, sendBuffer + sendIndex, outgoingPacket.packet_size);    

            // Check if this is the first packet
            if (sendIndex == 0) {
                outgoingPayload.first_packet = true;
            } else outgoingPayload.first_packet = false;

            // Check if this is the final packet
            if ((sendIndex + outgoingPacket.packet_size) >= sendBufferSize) {
                outgoingPayload.final_packet = true;
            } else outgoingPayload.final_packet = false;

            // Packet number is incremented in send callback if last packet was delivered successfully.
            // sendIndex is incremented similarily
            outgoingPacket.packet_number = send_packet_number;
        }
    } else outgoingPacket = emptyPacket; // Send empty packet by default

    // Next, finish loading the payload and send the data:
    outgoingPayload.packet = outgoingPacket;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingPayload, sizeof(outgoingPayload));

    // Check for errors
    if (result == ESP_OK) {
        Serial.print(millis()); Serial.print(": ");
        Serial.print("Pkt #"); Serial.print(outgoingPacket.packet_number);
        Serial.print(" sent via WiFi ("); Serial.print(outgoingPacket.packet_size);
        Serial.print(" bytes, "); Serial.print(sendBufferSize - sendIndex - outgoingPacket.packet_size);
        Serial.println(" remain):"); Serial.println(outgoingPacket.packet_data);
        Serial.print(outgoingPayload.first_packet ? "(first packet)" : "");
        Serial.println(outgoingPayload.final_packet ? "(final packet)" : "");

        /*
        Serial.println("Sent with success");
        Serial.println("PAYLOAD SENT:");
        Serial.print("First packet: "); Serial.println(outgoingPayload.first_packet ? "TRUE" : "FALSE");
        Serial.print("Final packet: "); Serial.println(outgoingPayload.final_packet ? "TRUE" : "FALSE");
        Serial.print("Packet data: "); Serial.println(outgoingPacket.packet_data);
        Serial.print("Packet number: "); Serial.println(outgoingPacket.packet_number);
        Serial.print("Packet size: "); Serial.println(outgoingPacket.packet_size);
        Serial.print("Total transmission size: "); Serial.println(outgoingPacket.data_size);
        Serial.print("Send index: "); Serial.println(sendIndex);
        Serial.print("sendBufferSize: "); Serial.println(sendBufferSize);
        */


    }
    else {
        Serial.println("Error sending the data.");
    }
}

bool espnow_WiFi_check_finished_receiving() {
    // Routine for when message has been fully received
    // This flag is set in 'espnow_WiFi_recv_payload()'

    if (finishedReceiving) {
        recvIndex = 0;
        receiving = false;

        if (recvBuffer != NULL) {
            recvBuffer[recvBufferSize] = '\0'; // Append the null terminator
        }
        
        finishedReceiving = false;
        return true;
    }

    if (!receiving){
        return true;
    }
    
    return false;
}

void espnow_WiFi_recv_payload() {

    /*
     *
     * [GPS DATA IS PROCESSED FROM THE PAYLOAD HERE]
     * Stored in 'incomingPayload.GPS_data' as a char array
     *
     */


    // If this is the first packet of a new transmission
    if (incomingPayload.first_packet) {
        receiving = true;
        recvBufferSize = incomingPacket.data_size;

        // Allocate data to receive buffer 
        recvBuffer = (char*)malloc(recvBufferSize + sizeof('\0'));

        if (!recvBuffer) {
            Serial.println("Memory allocation failed");
        }
    }

    // Overflow handling
    bool OVERFLOW = ((recvIndex + incomingPacket.packet_size) > MAX_DATA_SIZE);
    OVERFLOW = OVERFLOW || ((recvIndex + incomingPacket.packet_size) > recvBufferSize);

    // If there's no overflow, copy the received packet to recvBuffer
    if (!OVERFLOW) {
        memcpy(recvBuffer + recvIndex, incomingPacket.packet_data, incomingPacket.packet_size);
        recvIndex += incomingPacket.packet_size;
    } else {
        Serial.println("ERR: receive buffer overflow");
        finishedReceiving = true;
        espnow_WiFi_check_finished_receiving();
    }

    if (incomingPayload.final_packet) {
        receiving = false;
        finishedReceiving = true;
    }
}

// ESP-NOW send callback function
void sendWiFiCallback(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print(millis());
  Serial.print(": Payload delivered?\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
  Serial.println();
  if (status == 0) {
    success = "Delivery Success";
    sendIndex += outgoingPacket.packet_size; // If data was delivered successfully, send the next packet
    send_packet_number++; // If data was delivered successfully, send the next packet
    if (outgoingPayload.final_packet) {

        send_packet_number = 0;
        outgoingPacket.packet_number = 0;
        sendIndex = 0;
        outgoingPayload.final_packet = false;

        Serial.print(millis()); Serial.println(": Finished transmitting:");
        Serial.println(sendBuffer);
        Serial.println();
        finishedSending = true;
        sending = false;
        freeSendBuffer();

    }
  }
  else {
    success = "Delivery Fail";
  }
}

// ESP-NOW receive callback function
void recvWiFiCallback(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&incomingPayload, incomingData, sizeof(incomingPayload));
    
    Serial.print(millis()); Serial.print(": ");

    if (!(incomingPayload.packet.packet_data == emptyPacket.packet_data)) {

        incomingPacket = incomingPayload.packet;
        Serial.print("Pkt #"); Serial.print(incomingPacket.packet_number);
        Serial.print(" receieved via WiFi ("); Serial.print(incomingPacket.packet_size);
        Serial.print(" bytes, "); Serial.print(recvBufferSize - recvIndex - incomingPacket.packet_size);
        Serial.println(" remain):"); Serial.println(incomingPacket.packet_data);
        Serial.print(incomingPayload.first_packet ? "(first packet)" : "");
        Serial.println(incomingPayload.final_packet ? "(final packet)" : "");

    } else {

        Serial.println("Empty packet received.");       
    }
    
    Serial.println();

    // Except case where data is sent in only one packet long.
    if (!(incomingPacket.data_size == incomingPacket.packet_size)) {
         espnow_WiFi_recv_payload();
    }
}

// Start sending data

bool espnow_WiFi_send_data(uint8_t* broadcastAddress, char* data_to_send) {
    if (!sending) {

        sendBufferSize = strlen(data_to_send) + 1;

        // Case where message is only one packet
        // Send immediately, don't send piecewise.
        //if (sendBufferSize < PACKET_SIZE) {
        if (sendBufferSize < PACKET_SIZE) {

            // Load packet
            outgoingPacket.packet_number = 0;
            outgoingPacket.packet_size = sendBufferSize;
            outgoingPacket.data_size = sendBufferSize;
            memcpy(outgoingPacket.packet_data, data_to_send, outgoingPacket.packet_size);    

            // Load payload
            strcpy(outgoingPayload.GPS_data, placeholder_GPS_data);
            outgoingPayload.packet = outgoingPacket;
            outgoingPayload.first_packet = true;
            outgoingPayload.final_packet = true;
            outgoingPayload.payload_number++;

            // Send payload
            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingPayload, sizeof(outgoingPayload));

            // Check for errors
            if (result == ESP_OK) {
                Serial.println("Sent with success");
            }
            else {
                Serial.println("Error sending the data");
            }

            outgoingPayload.first_packet = false;
            outgoingPayload.final_packet = false;

        } else {
            sendBuffer = (char*)malloc(sendBufferSize);

            if (!sendBuffer) {
                Serial.println("Memory allocation failed");
                return false;
            }
            
            memcpy(sendBuffer,data_to_send,sendBufferSize);

            sending = true;
        }
        return true;
    }
    return false;
}

// Update global variables initialization to support dynamic allocation
void WiFi_Packet_Handling_init() {

    // Set flags and indices
    recvIndex = 0;
    sendIndex = 0;
    send_packet_number = 0;

    sending = false;
    receiving = false;
    finishedSending = false;
    finishedReceiving = false;

    // Load empty packet
    strcpy(emptyPacket.packet_data, "");
    emptyPacket.packet_number = 0;
    emptyPacket.packet_size = 0;
}

// Free dynamically allocated memory
void freeSendBuffer() {
    if (sendBuffer) {
        free(sendBuffer);
        sendBuffer = NULL;
    }
}

void freeRecvBuffer() {
    if (recvBuffer) {
        free(recvBuffer);
        recvBuffer = NULL;
    }
}

/*
void exampleLoop() {

    espnow_WiFi_send_payload();
    
    if (espnow_WiFi_check_finished_receiving()) {
        
        Serial.print("Transmission Received: ");
        Serial.println(recvBuffer);

        Serial.println("GPS data: ");
        Serial.println(incomingPayload.GPS_data);

        //uart_write(recvBuffer);

        freeRecvBuffer();
    }


    delay(1000);
    
}
*/