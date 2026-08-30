#pragma once
#include <cstdint>

// Servo
constexpr unsigned int SERVO_PIN = 15;

constexpr int SERVO_CENTER = 90;
constexpr int SERVO_LEFT = 60;
constexpr int SERVO_RIGHT = 120;

constexpr int SERVO_CENTER_OFFSET = -1;

constexpr unsigned int SERVO_MIN_PULSE_US = 500;
constexpr unsigned int SERVO_MAX_PULSE_US = 2500;

// Motor
constexpr unsigned int MOTOR_C1 = 6;
constexpr unsigned int MOTOR_C2 = 7;

// PWM
constexpr unsigned int MOTOR_PWM_TOP = 1000;
constexpr unsigned int MOTOR_PWM_FREQUENCY_HZ = 20000;

// Encoder
constexpr unsigned int ENCODER_A = 10;
constexpr unsigned int ENCODER_B = 11;

// Encoder settings
constexpr float ENCODER_COUNTS_PER_REVOLUTION = 600.0f;
constexpr unsigned int ENCODER_UPDATE_PERIOD_MS = 100;

// =====================================================
// BNO085 I2C0
// =====================================================

constexpr unsigned int IMU_SDA_PIN = 4;
constexpr unsigned int IMU_SCL_PIN = 5;

constexpr unsigned int IMU_I2C_BAUDRATE = 400000;

constexpr unsigned int BNO085_ADDRESS = 0x4A;

// =====================================================
// RASPBERRY PI 5 COMMUNICATION I2C1
// =====================================================

constexpr unsigned int PI_SDA_PIN = 2;
constexpr unsigned int PI_SCL_PIN = 3;

constexpr unsigned int PI_I2C_BAUDRATE = 400000;

// Pico slave address chosen for Pi communication
constexpr unsigned int PICO_I2C_ADDRESS = 0x39;