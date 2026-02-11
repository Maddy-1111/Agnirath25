#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H

#include <stdint.h>

// The global dictionary array (Matches Input_Order size in JSON)
extern float final_data[111]; 

// Structure for CAN Decoding Rules
struct CAN_Rule {
    uint32_t id;       // CAN ID
    uint16_t index;    // Target index in final_data
    bool is_float;     // True if native float (:1)
    float factor;      // Scaling factor
    float offset;      // Scaling offset
    uint8_t count;     // Number of signals in message
};

// Structure for Binary Output Formatting
struct Output_Field {
    uint16_t index; 
    const char* type; // "float16", "float32", "int16", "int32", "custom"
};

// --- CAN RULE TABLE (Generated from JSON + DBC) ---
const CAN_Rule can_rules[] = {
    {1537, 0,  false, 0.1f, 0.0f, 2}, // CMU1 Temps
    {1540, 2,  false, 0.1f, 0.0f, 2}, // CMU2
    {1543, 4,  false, 0.1f, 0.0f, 2}, // CMU3
    {1546, 6,  false, 0.1f, 0.0f, 2}, // CMU4
    {1549, 8,  false, 0.1f, 0.0f, 2}, // CMU5
    {1780, 10, true,  1.0f, 0.0f, 1}, // SOC Ah
    {1783, 11, false, 1.0f, 0.0f, 2}, // Precharge Flags
    {1786, 13, true,  1.0f, 0.0f, 2}, // Pack V/I
    {1787, 15, false, 1.0f, 0.0f, 1}, // BMS Flags (Bitmask)
    {1026, 58, true,  1.0f, 0.0f, 2}, // MC Bus V/I
    {1027, 60, true,  1.0f, 2},       // Velocity
    {1696, 67, true,  1.0f, 2}        // MPPT1 Input
};

// --- OUTPUT ORDER A (Fast Packet) ---
const Output_Field order_A[] = {
    {10, "float32"}, {13, "float32"}, {14, "float32"}, {58, "float32"}, {59, "float32"},
    {60, "float32"}, {61, "float32"}, {62, "float32"}, {63, "float32"}, {67, "float32"},
    {68, "float32"}, {69, "float32"}, {70, "float32"}, {74, "float32"}, {75, "float32"},
    {76, "float32"}, {77, "float32"}, {81, "float32"}, {82, "float32"}, {83, "float32"},
    {84, "float32"}, {88, "float32"}, {89, "float32"}, {90, "float32"}, {91, "float32"},
    {95, "float32"}, {96, "float32"}, {97, "float32"}, {98, "float32"}, {99, "float32"},
    {100, "float32"}, {101, "float32"}, {0xFFFF, "custom-88"} // 0xFFFF = Flags
};

// --- OUTPUT ORDER B (Slow Packet) ---
const Output_Field order_B[] = {
    {0, "float16"}, {1, "float16"}, {2, "float16"}, {3, "float16"}, {4, "float16"},
    {5, "float16"}, {6, "float16"}, {7, "float16"}, {8, "float16"}, {9, "float16"},
    {64, "float16"}, {65, "float16"}, {66, "float16"}, {71, "float16"}, {72, "float16"},
    {16, "float16"}, {17, "float16"}, {18, "float16"}, {19, "float16"}, {20, "float16"},
    {102, "int16"}, {103, "int16"}, {104, "int16"}, {105, "int16"}, {106, "int16"},
    {107, "int16"}, {108, "int32"}, {109, "int16"}, {0xFFFF, "custom-8"}
};

const int RULE_COUNT = sizeof(can_rules) / sizeof(CAN_Rule);
const int A_COUNT = sizeof(order_A) / sizeof(Output_Field);
const int B_COUNT = sizeof(order_B) / sizeof(Output_Field);

#endif