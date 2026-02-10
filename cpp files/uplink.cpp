#include "uplink.h"
#include "telemetry.h"

// The magic number used to mark the start of a new packet
const uint8_t HEADER[] = {0xDE, 0xAD, 0xBE, 0xEF};

// External reference to the global telemetry data populated by can_handler.cpp
extern TelemetryData telemetry;

void init_uplink(long baud_rate) {
    // Replaces: self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    Serial2.begin(baud_rate, SERIAL_8N1, 17, 16); // RX pin 17, TX pin 16 (Standard ESP32)
}

uint16_t calculate_crc16(const uint8_t *data, size_t len) {
    // Implements CRC-16-CCITT (XModem) with poly 0x1021
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void uplink_task(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(100); // 10Hz transmission rate
    
    while (1) {
        // 1. Prepare Buffer
        // We calculate the total size: Header(4) + Length(2) + Payload(struct size) + CRC(2)
        uint16_t payload_size = sizeof(TelemetryData);
        uint8_t packet[4 + 2 + payload_size + 2];
        
        // 2. Add Header
        memcpy(packet, HEADER, 4);
        
        // 3. Add Length (Little Endian)
        packet[4] = payload_size & 0xFF;
        packet[5] = (payload_size >> 8) & 0xFF;
        
        [cite_start]// 4. Copy Telemetry Data Payload [cite: 20]
        // We use memcpy for a thread-safe snapshot of the struct
        memcpy(&packet[6], &telemetry, payload_size);
        
        // 5. Calculate and Add CRC
        // CRC is calculated over the length and payload bytes
        uint16_t crc = calculate_crc16(&packet[4], payload_size + 2);
        packet[6 + payload_size] = crc & 0xFF;
        packet[7 + payload_size] = (crc >> 8) & 0xFF;
        
        // 6. Transmit over Serial2
        Serial2.write(packet, sizeof(packet));
        
        // Wait for the next interval
        vTaskDelay(xDelay);
    }
}