#pragma once

void motor_init();

void motor_forward(float speed_percent);
void motor_reverse(float speed_percent);

void motor_stop();
void motor_brake();