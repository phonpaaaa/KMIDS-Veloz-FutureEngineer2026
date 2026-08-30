#pragma once

bool imu_init();
bool imu_update();

float imu_get_yaw_rad();
float imu_get_pitch_rad();
float imu_get_roll_rad();

float imu_get_yaw_deg();
float imu_get_pitch_deg();
float imu_get_roll_deg();

bool imu_is_ready();