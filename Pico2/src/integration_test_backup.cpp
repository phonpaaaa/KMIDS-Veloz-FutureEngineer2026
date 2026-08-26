#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"

#include "config.h"
#include "encoder.h"
#include "imu.h"
#include "motor.h"
#include "servo.h"

// Run sensor updates and print telemetry for a fixed duration.
void monitor_robot(uint32_t duration_ms)
{
    const absolute_time_t start_time = get_absolute_time();
    absolute_time_t last_print_time = get_absolute_time();

    while (
        absolute_time_diff_us(
            start_time,
            get_absolute_time()
        ) < static_cast<int64_t>(duration_ms) * 1000
    )
    {
        encoder_update();
        imu_update();

        const absolute_time_t now = get_absolute_time();

        if (
            absolute_time_diff_us(
                last_print_time,
                now
            ) >= 100000
        )
        {
            printf(
                "Encoder: %ld | CPS: %.2f | RPM: %.2f | "
                "Yaw: %.2f | Pitch: %.2f | Roll: %.2f\n",
                static_cast<long>(encoder_get_count()),
                encoder_get_counts_per_second(),
                encoder_get_rpm(),
                imu_get_yaw_deg(),
                imu_get_pitch_deg(),
                imu_get_roll_deg()
            );

            last_print_time = now;
        }

        tight_loop_contents();
    }
}

int main()
{
    stdio_init_all();
    sleep_ms(2000);

    printf("\n====================================\n");
    printf("PICO 2 INTEGRATION TEST V3\n");
    printf("====================================\n");

    motor_init();
    servo_init();
    encoder_init();

    motor_stop();
    servo_center();
    encoder_reset();

    printf("Motor initialized\n");
    printf("Servo initialized\n");
    printf("Encoder initialized\n");

    if (!imu_init())
    {
        printf("ERROR: BNO085 initialization failed\n");

        motor_stop();

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf("BNO085 initialized\n");
    printf("Test begins in 3 seconds\n");

    sleep_ms(3000);

    while (true)
    {
        printf("\nStage 1: Forward 100%%, centered\n");

        encoder_reset();
        servo_center();

        // Use 100% first to verify the motor definitely starts.
        motor_forward(100.0f);

        monitor_robot(3000);

        printf("\nStage 2: Steering right while moving\n");

        servo_move_slow(
            SERVO_CENTER,
            SERVO_RIGHT,
            20
        );

        monitor_robot(2000);

        printf("\nStage 3: Steering left while moving\n");

        servo_move_slow(
            SERVO_RIGHT,
            SERVO_LEFT,
            20
        );

        monitor_robot(2000);

        printf("\nStage 4: Return to center\n");

        servo_move_slow(
            SERVO_LEFT,
            SERVO_CENTER,
            20
        );

        monitor_robot(2000);

        printf("\nStage 5: Brake and stop\n");

        motor_brake();
        sleep_ms(300);

        motor_stop();
        servo_center();

        monitor_robot(1000);

        printf("\nTest complete. Repeating in 5 seconds.\n");

        sleep_ms(5000);
    }
}