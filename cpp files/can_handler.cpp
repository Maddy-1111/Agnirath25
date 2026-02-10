#include "can_handler.h"
#include "telemetry.h"
#include "driver/twai.h"
#include <string.h>

extern TelemetryData telemetry;

void init_can_bus() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_22, GPIO_NUM_21, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); 
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();
}

// Internal helper to reduce code duplication for CMU Temperature IDs
void decode_cmu(const uint8_t* d, float* pcb, float* cell) {
    *pcb = ((d[5] << 8) | d[4]) * 0.1f;
    *cell = ((d[7] << 8) | d[6]) * 0.1f;
}

void can_rx_task(void *pvParameters) {
    twai_message_t msg;
    while (1) {
        if (twai_receive(&msg, pdMS_TO_TICKS(1)) == ESP_OK) {
            switch (msg.identifier) {
                
                // --- BMS / CMU TEMPERATURES ---
                case 1537: decode_cmu(msg.data, &telemetry.cmu1_pcb, &telemetry.cmu1_cell); break;
                case 1540: decode_cmu(msg.data, &telemetry.cmu2_pcb, &telemetry.cmu2_cell); break;
                case 1543: decode_cmu(msg.data, &telemetry.cmu3_pcb, &telemetry.cmu3_cell); break;
                case 1546: decode_cmu(msg.data, &telemetry.cmu4_pcb, &telemetry.cmu4_cell); break;
                case 1549: decode_cmu(msg.data, &telemetry.cmu5_pcb, &telemetry.cmu5_cell); break;

                // --- BMS PACK & STATUS ---
                case 1780: memcpy(&telemetry.soc_ah, &msg.data[0], 4); memcpy(&telemetry.soc_percent, &msg.data[4], 4); break;
                case 1783: telemetry.precharge_stat = msg.data[0]; telemetry.precharge_state = msg.data[1]; telemetry.precharge_timer = (msg.data[7]*1.0f) + 10.0f; break;
                case 1786: memcpy(&telemetry.battery_voltage, &msg.data[0], 4); memcpy(&telemetry.battery_current, &msg.data[4], 4); break;
                case 1789: telemetry.hardware_ver = msg.data[4]; break; // Index 23 in main_node

                // --- BMS ERROR FLAG RE-MAPPING (The 8.0 to 1.0 Priority Logic) ---
                case 1787:
                    telemetry.bms_flags[0] = (msg.data[4] & 0x80) ? 8.0f : 0.0f; 
                    telemetry.bms_flags[1] = (msg.data[4] & 0x40) ? 7.0f : 0.0f;
                    telemetry.bms_flags[2] = (msg.data[4] & 0x20) ? 6.0f : 0.0f;
                    telemetry.bms_flags[3] = (msg.data[4] & 0x10) ? 5.0f : 0.0f;
                    telemetry.bms_flags[4] = (msg.data[4] & 0x08) ? 4.0f : 0.0f;
                    telemetry.bms_flags[5] = (msg.data[4] & 0x04) ? 3.0f : 0.0f;
                    telemetry.bms_flags[6] = (msg.data[4] & 0x02) ? 2.0f : 0.0f;
                    telemetry.bms_flags[7] = (msg.data[4] & 0x01) ? 1.0f : 0.0f;
                    break;

                // --- MOTOR CONTROLLER (Native Floats) ---
                case 1025: telemetry.mc_limit_flags = msg.data[0]; telemetry.mc_error_flags = msg.data[2]; break;
                case 1026: memcpy(&telemetry.mc_bus_v, &msg.data[0], 4); memcpy(&telemetry.mc_bus_i, &msg.data[4], 4); break;
                case 1027: memcpy(&telemetry.mc_vel_motor, &msg.data[0], 4); memcpy(&telemetry.mc_vel_vehicle, &msg.data[4], 4); break;
                case 1028: memcpy(&telemetry.mc_phase_b, &msg.data[0], 4); memcpy(&telemetry.mc_phase_c, &msg.data[4], 4); break;
                case 1035: memcpy(&telemetry.mc_temp_motor, &msg.data[0], 4); memcpy(&telemetry.mc_temp_heatsink, &msg.data[4], 4); break;
                case 1036: memcpy(&telemetry.mc_dsp_temp, &msg.data[0], 4); break;
                case 1038: memcpy(&telemetry.mc_odometer, &msg.data[0], 4); break;

                // --- ALL MPPT UNITS (Inputs, Outputs, Temps, Status)  ---
                // MPPT 1
                case 1696: memcpy(&telemetry.mppt1_vin, &msg.data[0], 4); memcpy(&telemetry.mppt1_iin, &msg.data[4], 4); break;
                case 1697: memcpy(&telemetry.mppt1_vout, &msg.data[0], 4); memcpy(&telemetry.mppt1_iout, &msg.data[4], 4); break;
                case 1698: memcpy(&telemetry.mppt1_temp_mos, &msg.data[0], 4); memcpy(&telemetry.mppt1_temp_ctrl, &msg.data[4], 4); break;
                case 1701: telemetry.mppt1_error = msg.data[4]; break;
                // MPPT 2
                case 1712: memcpy(&telemetry.mppt2_vin, &msg.data[0], 4); memcpy(&telemetry.mppt2_iin, &msg.data[4], 4); break;
                case 1713: memcpy(&telemetry.mppt2_vout, &msg.data[0], 4); memcpy(&telemetry.mppt2_iout, &msg.data[4], 4); break;
                case 1714: memcpy(&telemetry.mppt2_temp_mos, &msg.data[0], 4); memcpy(&telemetry.mppt2_temp_ctrl, &msg.data[4], 4); break;
                case 1717: telemetry.mppt2_error = msg.data[4]; break;
                // MPPT 3
                case 1728: memcpy(&telemetry.mppt3_vin, &msg.data[0], 4); memcpy(&telemetry.mppt3_iin, &msg.data[4], 4); break;
                case 1729: memcpy(&telemetry.mppt3_vout, &msg.data[0], 4); memcpy(&telemetry.mppt3_iout, &msg.data[4], 4); break;
                case 1730: memcpy(&telemetry.mppt3_temp_mos, &msg.data[0], 4); memcpy(&telemetry.mppt3_temp_ctrl, &msg.data[4], 4); break;
                case 1733: telemetry.mppt3_error = msg.data[4]; break;
                // MPPT 4
                case 1744: memcpy(&telemetry.mppt4_vin, &msg.data[0], 4); memcpy(&telemetry.mppt4_iin, &msg.data[4], 4); break;
                case 1745: memcpy(&telemetry.mppt4_vout, &msg.data[0], 4); memcpy(&telemetry.mppt4_iout, &msg.data[4], 4); break;
                case 1746: memcpy(&telemetry.mppt4_temp_mos, &msg.data[0], 4); memcpy(&telemetry.mppt4_temp_ctrl, &msg.data[4], 4); break;
                case 1749: telemetry.mppt4_error = msg.data[4]; break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}