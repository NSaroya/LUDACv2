# Long Distance Underwater and Aerial Drone Communication (LUDAC)

## Team Members

- Nikhil Saroya
- Xinkai Xu
- Ethan Wilson
- Antony Brittain

## Overview

The Long Distance Underwater Drone to Aerial Communication project (LUDAC) integrates LoRa and WiFi for long range and near range communication between a submersible and an aerial drone. It realizes long distance communication with a max transmission distance of 1.3km using the LoRa protocol. Near-range communication is realized with the ESPNOW WiFi protocol, which has the ability to transmit large file sizes. The board is configured to switch between WiFi and LoRa based on signal quality to conserve power. LUDAC boards have been benchmarked for performance criteria including power consumption, packet integrity, bitrate, transmission distance, packet delay, LoRa/WiFi switching delay, etc. Overall, LUDAC boards have exceeded or met most of the specification requirements. The proposed functionalities, along with the extended goals of large file transmission and power supply configurability have been achieved by the design.

## Contributors

- **Nikhil Saroya - main.cpp, LUDAC_GPS.cpp, LUDAC_WiFi.cpp** - April 2024
- **Xinkai Xu - LUDAC_LoRa.cpp, LUDAC_GPS.cpp** - April 2024
- **Ethan Wilson - LUDAC_WiFi_Duplex.cpp, LUDAC_WiFi.cpp** - April 2024
- **Antony Brittain - Test and Reliability** - April 2024

## HIGH LEVEL FUNCTIONALITIES

The primary goal of LUDAC transceivers, as requested by our client, is to maintain stable long-distance duplex communication between a surfaced submersible and the aerial drone dispatched for survey, rescue, or object retrieval tasks. The main types of data to be transmitted include: GPS coordinates of the submersible and aerial drone, relative distance, mission status (e.g., mission complete/in progress), and landing requests. When the submersible and aerial drone are in close proximity (i.e., in the order of meters), the transceivers should engage WiFi mode to support high bitrate communication. Image transmission via WiFi is also suggested as a stretch goal. When the aerial drone exceeds the range of WiFi communication, the Long-Range (LoRa) protocol will be engaged to realize long distance, low bitrate communication. The board is expected to be flexible with different types of power supplies (e.g., USB 5V/3.3V input, direct lithium ion battery input ranging from 10 to 14V), meanwhile maintain a low power consumption to avoid shortening mission time. 

## Firmware and Software Design

![Screenshot 2024-05-09 174457](https://github.com/NSaroya/LUDACv2/assets/156468713/260c746f-4a29-448f-97e1-7040bcef3ed7)

### Development environment
The firmware and software development is done with PlatformIO in the Espressif environment, an ESP32 proprietary development environment based on C/C++ framework. The Arduino framework is partially corporated with consent from our client, team members, and the ECE 491 instructors, as all LoRa/ESP-NOW libraries have Arduino dependencies. Also, by avoiding the poorly documented non-Arduino ESP-NOW libraries, this ensures smooth project handover to the client’s submersible R&D team. 

### Interface with the On-Board Processor
The communication between the ESP32 and Raspberry Pi is achieved by continuously polling the UART interface to check for incoming instructions. This polling-based approach ensures consistent handling of data and events at regular intervals, making it suitable for applications where real-time responsiveness is not critical or where the system resources are limited. When an instruction is received to send data from the master (Raspberry Pi) to the slave (ESP32), the data is read and stored in a local buffer. If the received data exceeds the static buffer size, the code dynamically resizes the buffer to accommodate the incoming data. This prevents buffer overflow errors and ensures the complete reception of the data. Conversely, when an instruction is received to send data from the slave (ESP32) to the master (Raspberry Pi), the available data is transmitted to the master. The polling mechanism ensures that the system regularly checks for and processes these data transmission requests.

The use of polling, rather than an interrupt-driven approach, is a deliberate design choice. Polling provides a more predictable and consistent data handling mechanism, which is important in applications where real-time responsiveness is not the primary concern. Interrupt-driven approaches can be more complex to implement and may introduce unpredictable delays or race conditions, especially in resource-constrained systems like the ESP32.

### LoRa half-duplexing

LoRa half-duplexing is realized through random scheduling with an average transmission interval of 2 seconds, as shown in the figure below.  This method does not involve acknowledgement of successful packet delivery/reception, but is confirmed to be virtually lossless from the benchmark results (to be discussed later) within the operation range. The firmware development is based on the Sandeep-Mistry-LoRa library with LUDAC application specific modifications. 

### ESP-NOW

ESP-NOW is a wireless communication protocol developed by Espressif that enables multiple devices equipped with ESP-series microcontrollers (like the ESP32 and ESP8266) to communicate with each other directly, without the need for a WiFi network [5]. This protocol operates on the 2.4 GHz WiFi band, offering a low-power, low-latency solution for various IoT applications. ESP-NOW is especially valuable in scenarios where devices need to exchange small amounts of data quickly and efficiently, such as sensor networks, home automation, and wearable electronics. Its ability to facilitate secure, peer-to-peer communication makes it an ideal for this project, which requires direct connection and coordination between multiple microcontrollers, minimal power consumption, and a simple network topology. 

### LoRa/ESP-NOW switching

Switching between LoRa and ESP-NOW protocols minimizes power consumption by ensuring that only one type of protocol is transmitting or receiving at a time. Switching takes advantage of the ESP32 packet delivery acknowledgement. This mechanism allows the sender to know whether the data was received successfully by the peer device. If the module callback function receives at least 90% success acknowledgements indicating successful delivery of packets, the system enables WiFi transmission mode, otherwise it enables LoRa transmission mode. This approach allows the system to dynamically adapt to changing environmental conditions and network connectivity, ensuring optimal power consumption while maintaining reliable data transmission. The switching function was initially designed to use an absolute distance calculation, but this resulted in measurement errors of up to 30% at the 1km mark.

### Transceiving Data Segmentation

Both ESP-NOW and LoRa communication protocols are limited by a ~250 byte payload size, so longer data is handled algorithmically via segmentation. In standard operation, payloads containing mission and GPS data are sent and received continuously, with space allocated in each payload for a packet of segmented data. This space is unused when the device is not transmitting long-form data. Upon receiving an instruction from the onboard computer to send long-form data, the firmware sets the ‘sending’ flag, which initiates the segmentation routine. The transmitting device first indicates to the intended recipient when it is about to begin segmented transmission by sending a payload indicating the length of the impending message and a flag called ‘first_packet’ set to true. This causes the other board to set its ‘receiving’ flag. The transmitter then begins loading each outgoing payload with a new packet of the data stored in ‘sendBuffer’. When a packet is successfully received by the other board, the firmware increments the ‘sendIndex’ pointer and transmits the next packet of data from the buffer. It repeats this process until the message is fully received by the other board, at which point the ‘sending’ flag is set to false. The final packet is indicated to the other board with a ‘final_packet’ flag. Upon receiving a packet from the other device, a callback function interrupts the main routine to load it into ‘recvBuffer’. When the buffer is full, the data is sent to the computer and cleared. At present this process is only implemented for ESP-NOW, but it could be extended to LoRa in future versions. 

## Code Dependencies

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

## Notes

- The firmware is designed to work with specific hardware configurations and may require adjustments for compatibility with different setups.
- To enable efficient communication between the LUDAC and the main computer, an asynchronous, event-driven communication technique has been implemented. This approach helps minimize waiting time and improve overall system efficiency, especially if the ESP32 microcontroller has other tasks to perform concurrently. To set up this communication, the GPIO_EVENT_TRIGGER can be configured to the desired hardware pin on the ESP32, which will then be used to signal the state of the ESP32 to the main computer. A sample implementation of this setup on the Raspberry Pi side is provided in the LUDAC_RPi_Sample.txt file.
- In the firmware, the WiFi and LoRa modules are currently sending sample information, including example_data_to_send_lora and example_data_to_send_wifi. To improve the functionality, this should be slightly modified to use the uartDataReceivedFromMaster variable instead. The required code for this modification is already provided in the firmware, and it just needs minor adjustments to make it work as intended.
- Debugging messages are printed using verbose printing macros for easier troubleshooting.

