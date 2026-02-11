#include "telemetry.h"
#include "driver/twai.h"
#include <Arduino.h>
#include "utils.h"
#include "config.h"

float final_data[111] = {0.0f};

void setup() {
    Serial2.begin(115200, SERIAL_8N1, 16, 17); // Telemetry Serial
    
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_22, GPIO_NUM_21, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();
}

void loop() {
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(1)) == ESP_OK) {
        decode_can_frame(msg);
    }

    if (msg.identifier == 1696) {
            Serial.print("MPPT A (ID: 1696) -> ");
            Serial.print("Input Voltage: ");
            Serial.print(final_data[67]);
            Serial.print(" V | Input Current: ");
            Serial.print(final_data[68]);
            Serial.println(" A");
        }

    static uint32_t last_tx = 0;
    if (millis() - last_tx > 1000) { // 100Hz
        send_uplink();
        last_tx = millis();
    }
}

'''void loop() {
    twai_message_t msg;
    
    // Check if a CAN message is received
    if (twai_receive(&msg, pdMS_TO_TICKS(1)) == ESP_OK) {
        decode_can_frame(msg);

        // --- NEW: Print MPPT A Data to Serial Monitor ---
        // MPPT A ID is 1696. It maps to indexes 67 and 68.
        if (msg.identifier == 1696) {
            Serial.print("MPPT A (ID: 1696) -> ");
            Serial.print("Input Voltage: ");
            Serial.print(final_data[67]);
            Serial.print(" V | Input Current: ");
            Serial.print(final_data[68]);
            Serial.println(" A");
        }
    }
        '''