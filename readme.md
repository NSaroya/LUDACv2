# LUDAC Firmware

## Overview

This repository contains the main firmware for the Long-distance Underwater Drone to Aerial Communication (LUDAC) project developed as part of the ECE 490/491 Capstone Design Project. This firmware facilitates communication between surfaced underwater submarines and aerial drones.

## Author

- **Nikhil Saroya - main.cpp, LUDAC_GPS.cpp, LUDAC_WiFi.cpp** - April 2024
- **Xinkai Xu - LUDAC_LoRa.cpp, LUDAC_GPS.cpp** - April 2024
- **Ethan Wilson - LUDAC_WiFi_Duplex.cpp, LUDAC_WiFi.cpp** - April 2024
- **Antony Brittain - Test and Reliability** - April 2024

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

- LUDAC_LoRa.h
- LUDAC_WiFi.h
- LUDAC_GPS.h
- LUDAC_WiFi_Duplex.h

Third-party Dependencies:
- Arduino.h
- LoRa.h
- WiFi.h
- esp_now.h
- esp_wifi.h
- NMEA_data.h
- Adafruit_GPS.h
- Adafruit_PMTK.h
- SoftwareSerial.h

## Configuration

Ensure that the most up-to-date configuration files are used in conjunction with this firmware.

## Notes

- The firmware is designed to work with specific hardware configurations and may require adjustments for compatibility with different setups.
- To enable efficient communication between the LUDAC and the main computer, an asynchronous, event-driven communication technique has been implemented. This approach helps minimize waiting time and improve overall system efficiency, especially if the ESP32 microcontroller has other tasks to perform concurrently. To set up this communication, the GPIO_EVENT_TRIGGER can be configured to the desired hardware pin on the ESP32, which will then be used to signal the state of the ESP32 to the main computer. A sample implementation of this setup on the Raspberry Pi side is provided in the LUDAC_RPi_Sample.txt file.
- In the firmware, the WiFi and LoRa modules are currently sending sample information, including example_data_to_send_lora and example_data_to_send_wifi. To improve the functionality, this should be slightly modified to use the uartDataReceivedFromMaster variable instead. The required code for this modification is already provided in the firmware, and it just needs minor adjustments to make it work as intended.
- Debugging messages are printed using verbose printing macros for easier troubleshooting.

