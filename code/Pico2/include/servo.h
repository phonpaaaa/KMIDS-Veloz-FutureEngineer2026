#pragma once

#include <cstdint>

void servo_init();

void servo_write(int angle);

void servo_move_slow(
    int start_angle,
    int end_angle,
    std::uint32_t delay_ms
);

void servo_center();