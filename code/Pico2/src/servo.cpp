#include "servo.h"

#include "config.h"

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

namespace
{
int clamp_angle(int angle)
{
    if (angle < 0)
    {
        return 0;
    }

    if (angle > 180)
    {
        return 180;
    }

    return angle;
}
}

void servo_init()
{
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);

    const uint slice =
        pwm_gpio_to_slice_num(SERVO_PIN);

    pwm_config config =
        pwm_get_default_config();

    // Make one PWM count equal approximately 1 microsecond.
    const float divider =
        static_cast<float>(clock_get_hz(clk_sys)) /
        1000000.0f;

    pwm_config_set_clkdiv(
        &config,
        divider
    );

    // 20,000 microseconds = 20 ms = 50 Hz.
    pwm_config_set_wrap(
        &config,
        19999
    );

    pwm_init(
        slice,
        &config,
        true
    );

    servo_center();
}

void servo_write(int requested_angle)
{
    int calibrated_angle =
        requested_angle + SERVO_CENTER_OFFSET;

    calibrated_angle =
        clamp_angle(calibrated_angle);

    const std::uint16_t pulse_us =
        SERVO_MIN_PULSE_US +
        static_cast<std::uint16_t>(
            (SERVO_MAX_PULSE_US -
             SERVO_MIN_PULSE_US) *
            calibrated_angle /
            180
        );

    pwm_set_gpio_level(
        SERVO_PIN,
        pulse_us
    );
}

void servo_move_slow(
    int start_angle,
    int end_angle,
    std::uint32_t delay_ms
)
{
    if (start_angle < end_angle)
    {
        for (
            int angle = start_angle;
            angle <= end_angle;
            angle++
        )
        {
            servo_write(angle);
            sleep_ms(delay_ms);
        }
    }
    else
    {
        for (
            int angle = start_angle;
            angle >= end_angle;
            angle--
        )
        {
            servo_write(angle);
            sleep_ms(delay_ms);
        }
    }
}

void servo_center()
{
    servo_write(SERVO_CENTER);
}