#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include "driver/twai.h"

// Hardware Initialization
void init_peripherals();

// Data Processing Logic
void decode_can_frame(twai_message_t msg);

// Uplink & Communication
void send_uplink();
uint16_t calculate_crc(const uint8_t *data, size_t len);

// Data Converters
uint16_t float32_to_float16(float value);
int pack_all_flags(uint8_t* buffer);

#endif