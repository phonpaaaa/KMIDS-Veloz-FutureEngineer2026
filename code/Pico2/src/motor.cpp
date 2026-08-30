#include "motor.h"

#include "config.h"

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

namespace
{
float clamp_percent(float speed_percent)
{
    if (speed_percent < 0.0f)
    {
        return 0.0f;
    }

    if (speed_percent > 100.0f)
    {
        return 100.0f;
    }

    return speed_percent;
}

uint16_t percent_to_pwm(float speed_percent)
{
    speed_percent = clamp_percent(speed_percent);

    return static_cast<uint16_t>(
        speed_percent *
        static_cast<float>(MOTOR_PWM_TOP - 1) /
        100.0f
    );
}
}

void motor_init()
{
    gpio_set_function(MOTOR_C1, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_C2, GPIO_FUNC_PWM);

    const uint slice = pwm_gpio_to_slice_num(MOTOR_C1);

    pwm_config config = pwm_get_default_config();

    const float divider =
        static_cast<float>(clock_get_hz(clk_sys)) /
        static_cast<float>(
            MOTOR_PWM_FREQUENCY_HZ * MOTOR_PWM_TOP
        );

    pwm_config_set_clkdiv(&config, divider);
    pwm_config_set_wrap(&config, MOTOR_PWM_TOP - 1);

    pwm_init(slice, &config, true);

    motor_stop();
}

void motor_forward(float speed_percent)
{
    const uint16_t pwm_level =
        percent_to_pwm(speed_percent);

    pwm_set_gpio_level(MOTOR_C1, pwm_level);
    pwm_set_gpio_level(MOTOR_C2, 0);
}

void motor_reverse(float speed_percent)
{
    const uint16_t pwm_level =
        percent_to_pwm(speed_percent);

    pwm_set_gpio_level(MOTOR_C1, 0);
    pwm_set_gpio_level(MOTOR_C2, pwm_level);
}

void motor_stop()
{
    // Both LOW means coast.
    pwm_set_gpio_level(MOTOR_C1, 0);
    pwm_set_gpio_level(MOTOR_C2, 0);
}

void motor_brake()
{
    // Both HIGH means active brake.
    pwm_set_gpio_level(MOTOR_C1, MOTOR_PWM_TOP - 1);
    pwm_set_gpio_level(MOTOR_C2, MOTOR_PWM_TOP - 1);
}