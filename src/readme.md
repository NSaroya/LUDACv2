# LUDAC Firmware

## Overview

This repository contains the main firmware for the Long-distance Underwater Drone to Aerial Communication (LUDAC) project developed as part of the ECE 490/491 Capstone Design Project. This firmware facilitates communication between underwater drones and aerial devices.

## Author

- **Nikhil Saroya** - April 2024

## Team Members

- Nikhil Saroya
- Xinkai Xu
- Ethan Wilson
- Antony Brittain

## Description

The main firmware (`main.cpp`) serves as the core software component for the LUDAC project. It includes functionalities for LoRa communication, Wi-Fi communication, GPS data handling, and UART communication with external devices such as a Raspberry Pi.

## Functions

1. `setup`: Initializes various modules and components required for the operation of the firmware.
2. `loop`: Contains the main execution loop for the firmware, handling LoRa and Wi-Fi communication, GPS data retrieval, and other tasks.
3. `GetPeerRSSI`: Retrieves the Received Signal Strength Indication (RSSI) of a peer device in the Wi-Fi network.
4. `generate_LoRaTx_Buffer`: Generates a transmission buffer for LoRa communication, combining GPS data and UART data.
5. `Parse_LoRaRx_Buffer`: Parses the received LoRa buffer into separate GPS and LoRa data segments.
6. `printGPSInfo`: Prints GPS information for debugging purposes.
7. `convertUTCToMST`: Converts Coordinated Universal Time (UTC) to Mountain Standard Time (MST).
8. `parseGGA`: Parses GPS data in the GGA format.

## Dependencies

- Arduino.h
- HardwareSerial.h
- LUDAC_LoRa.h
- LUDAC_WiFi.h
- LUDAC_GPS.h
- driver/uart.h
- freertos/FreeRTOS.h
- freertos/task.h

## Configuration

Ensure that the most up-to-date configuration files are used in conjunction with this firmware.

## Notes

- The firmware is designed to work with specific hardware configurations and may require adjustments for compatibility with different setups.
- Debugging messages are printed using verbose printing macros for easier troubleshooting.

