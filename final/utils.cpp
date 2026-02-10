#include "telemetry_data.h"
#include "driver/twai.h"
#include <Arduino.h>

// Convert Float32 to Half-Precision (Float16) for Type B Packets
uint16_t float32_to_float16(float value) {
    uint32_t i;
    memcpy(&i, &value, 4);
    uint16_t s = (i >> 16) & 0x8000;
    uint16_t e = ((i >> 23) & 0xff) - (127 - 15);
    uint16_t m = (i >> 13) & 0x3ff;
    if (e <= 0) return s;
    if (e >= 31) return s | 0x7c00;
    return s | (e << 10) | m;
}

void decode_can_frame(twai_message_t msg) {
    for (int i = 0; i < RULE_COUNT; i++) {
        if (msg.identifier == can_rules[i].id) {
            if (can_rules[i].is_float) {
                memcpy(&final_data[can_rules[i].index], &msg.data[0], can_rules[i].count * 4);
            } else {
                uint16_t r1 = (msg.data[5] << 8) | msg.data[4];
                final_data[can_rules[i].index] = (r1 * can_rules[i].factor) + can_rules[i].offset;
                if (can_rules[i].count == 2) {
                    uint16_t r2 = (msg.data[7] << 8) | msg.data[6];
                    final_data[can_rules[i].index + 1] = (r2 * can_rules[i].factor) + can_rules[i].offset;
                }
            }
            return;
        }
    }
}

uint16_t calculate_crc(const uint8_t *data, size_t len) {
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

void send_uplink() {
    static uint32_t count = 0;
    count++;
    
    char type = (count % 10 == 0) ? 'B' : 'A';
    const Output_Field* order = (type == 'B') ? order_B : order_A;
    int size = (type == 'B') ? B_COUNT : A_COUNT;

    uint8_t payload[512];
    int p = 0;

    for (int i = 0; i < size; i++) {
        if (order[i].index == 0xFFFF) {
            // Placeholder for bit-packing logic (Flags)
            payload[p++] = 0x00; 
        } else {
            float val = final_data[order[i].index];
            if (strcmp(order[i].type, "float16") == 0) {
                uint16_t h = float32_to_float16(val);
                memcpy(&payload[p], &h, 2); p += 2;
            } else if (strcmp(order[i].type, "int16") == 0) {
                int16_t s = (int16_t)val;
                memcpy(&payload[p], &s, 2); p += 2;
            } else {
                memcpy(&payload[p], &val, 4); p += 4;
            }
        }
    }

    uint8_t header[] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint16_t len = p;
    
    // Checksum over Type + Payload
    uint8_t crc_in[513];
    crc_in[0] = type;
    memcpy(&crc_in[1], payload, len);
    uint16_t crc = calculate_crc(crc_in, len + 1);

    Serial2.write(header, 4);
    Serial2.write((uint8_t*)&len, 2);
    Serial2.write((uint8_t*)&crc, 2);
    Serial2.write(type);
    Serial2.write(payload, len);
}