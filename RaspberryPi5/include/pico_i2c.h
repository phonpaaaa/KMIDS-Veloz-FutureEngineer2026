#pragma once

#include <cstdint>

struct PicoCommand
{
    std::int8_t speed_percent;
    std::uint8_t steering_angle;
    std::uint8_t emergency_stop;
};

struct PicoTelemetry
{
    std::int32_t encoder_count;

    std::int16_t rpm_x10;

    std::int16_t yaw_x10;
    std::int16_t pitch_x10;
    std::int16_t roll_x10;
};

bool pico_i2c_init();

bool pico_i2c_send_command(
    const PicoCommand& command
);

bool pico_i2c_read_telemetry(
    PicoTelemetry& telemetry
);