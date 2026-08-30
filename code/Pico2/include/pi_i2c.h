#pragma once

#include <stdint.h>

// ==========================================
// COMMAND: Raspberry Pi 5 -> Pico 2
// ==========================================

struct PiCommand
{
    int8_t speed_percent;
    uint8_t steering_angle;
    uint8_t emergency_stop;
};

// ==========================================
// TELEMETRY: Pico 2 -> Raspberry Pi 5
// ==========================================

struct PicoTelemetry
{
    int32_t encoder_count;

    int16_t rpm_x10;

    int16_t yaw_x10;
    int16_t pitch_x10;
    int16_t roll_x10;
};

// ==========================================
// I2C
// ==========================================

void pi_i2c_init();
void pi_i2c_update();

// Receive command from Pi
PiCommand pi_i2c_get_command();

bool pi_i2c_command_received();
void pi_i2c_clear_command_flag();

// Provide telemetry for Pi to read
void pi_i2c_set_telemetry(
    const PicoTelemetry& telemetry
);