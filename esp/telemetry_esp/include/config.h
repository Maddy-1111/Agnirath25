#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- Serial Configuration ---
#define SERIAL_BAUD 115200       // Replicates BAUD_RATE
#define SERIAL_TX_PIN 16         // Standard ESP32 UART2 TX
#define SERIAL_RX_PIN 17         // Standard ESP32 UART2 RX

// --- CAN (TWAI) Configuration ---
#define CAN_TX_GPIO GPIO_NUM_21  // CAN TX Pin
#define CAN_RX_GPIO GPIO_NUM_22  // CAN RX Pin
#define CAN_SPEED TWAI_TIMING_CONFIG_500KBITS() // Replicates bitrate=500000

// --- Timing & Logic ---
#define UPLINK_INTERVAL_MS 100   // 10Hz transmission
#define DICTIONARY_SIZE 111      // Total elements in packet_structure.json
#define MAGIC_HEADER 0xDEADBEEF  // Packet Start Marker

#endif