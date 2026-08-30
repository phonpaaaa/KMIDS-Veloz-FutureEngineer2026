#include "encoder.h"

#include "config.h"

#include "hardware/sync.h"
#include "pico/stdlib.h"

namespace
{
volatile int32_t encoder_count = 0;

int32_t previous_count = 0;

float counts_per_second = 0.0f;
float motor_rpm = 0.0f;

absolute_time_t last_update_time;

void encoder_callback(uint gpio, uint32_t events)
{
    (void)events;

    if (gpio != ENCODER_A)
    {
        return;
    }

    const bool channel_a = gpio_get(ENCODER_A);
    const bool channel_b = gpio_get(ENCODER_B);

    if (channel_a == channel_b)
    {
        encoder_count++;
    }
    else
    {
        encoder_count--;
    }
}
}

void encoder_init()
{
    gpio_init(ENCODER_A);
    gpio_set_dir(ENCODER_A, GPIO_IN);
    gpio_pull_up(ENCODER_A);

    gpio_init(ENCODER_B);
    gpio_set_dir(ENCODER_B, GPIO_IN);
    gpio_pull_up(ENCODER_B);

    gpio_set_irq_enabled_with_callback(
        ENCODER_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_callback
    );

    encoder_reset();
}

void encoder_reset()
{
    const uint32_t interrupt_state =
        save_and_disable_interrupts();

    encoder_count = 0;

    restore_interrupts(interrupt_state);

    previous_count = 0;
    counts_per_second = 0.0f;
    motor_rpm = 0.0f;

    last_update_time = get_absolute_time();
}

int32_t encoder_get_count()
{
    const uint32_t interrupt_state =
        save_and_disable_interrupts();

    const int32_t count = encoder_count;

    restore_interrupts(interrupt_state);

    return count;
}

void encoder_update()
{
    const absolute_time_t now =
        get_absolute_time();

    const int64_t elapsed_us =
        absolute_time_diff_us(
            last_update_time,
            now
        );

    const int64_t required_period_us =
        static_cast<int64_t>(
            ENCODER_UPDATE_PERIOD_MS
        ) * 1000;

    if (elapsed_us < required_period_us)
    {
        return;
    }

    const int32_t current_count =
        encoder_get_count();

    const int32_t count_difference =
        current_count - previous_count;

    const float elapsed_seconds =
        static_cast<float>(elapsed_us) /
        1000000.0f;

    counts_per_second =
        static_cast<float>(count_difference) /
        elapsed_seconds;

    motor_rpm =
        (
            counts_per_second /
            ENCODER_COUNTS_PER_REVOLUTION
        ) *
        60.0f;

    previous_count = current_count;
    last_update_time = now;
}

float encoder_get_counts_per_second()
{
    return counts_per_second;
}

float encoder_get_rpm()
{
    return motor_rpm;
}