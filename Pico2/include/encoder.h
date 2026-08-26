#pragma once

#include <stdint.h>

void encoder_init();
void encoder_reset();
void encoder_update();

int32_t encoder_get_count();
float encoder_get_counts_per_second();
float encoder_get_rpm();