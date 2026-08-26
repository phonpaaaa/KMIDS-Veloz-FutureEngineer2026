#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"

#include "config.h"
#include "encoder.h"
#include "imu.h"
#include "motor.h"
#include "pi_i2c.h"
#include "servo.h"

// ==========================================================
// VELOZ PICO 2 CONTROLLER
//
// Raspberry Pi 5 = MASTER / BRAIN
// Pico 2         = I2C SLAVE / HARDWARE CONTROLLER
//
// Raspberry Pi decides:
//     - motor speed
//     - steering angle
//
// Pico executes:
//     - motor
//     - servo
//     - encoder
//     - IMU
//
// If Pi communication disappears:
//     - MOTOR STOP
//     - SERVO CENTER
// ==========================================================


// ==========================================================
// SAFETY
// ==========================================================

constexpr uint32_t COMMAND_TIMEOUT_MS = 300;


// ==========================================================
// LOOP TIMING
// ==========================================================

// Main controller loop
constexpr uint32_t MAIN_LOOP_MS = 1;

// Telemetry update
constexpr uint32_t TELEMETRY_INTERVAL_MS = 20;

// Debug printing
constexpr uint32_t DEBUG_INTERVAL_MS = 500;


// ==========================================================
// STEERING
// ==========================================================

constexpr int STEERING_MIN = 40;
constexpr int STEERING_CENTER = 90;
constexpr int STEERING_MAX = 140;


// ==========================================================
// STATE
// ==========================================================

uint32_t last_command_time_ms = 0;
uint32_t last_telemetry_time_ms = 0;
uint32_t last_debug_time_ms = 0;

bool watchdog_active = true;

// Last actual hardware command
int last_speed = 0;
int last_steering = STEERING_CENTER;


// ==========================================================
// HELPERS
// ==========================================================

int clamp_steering(int angle)
{
    if (angle < STEERING_MIN)
    {
        return STEERING_MIN;
    }

    if (angle > STEERING_MAX)
    {
        return STEERING_MAX;
    }

    return angle;
}


// ==========================================================
// MOTOR COMMAND
// ==========================================================

void apply_motor_command(int speed)
{
    // Don't keep rewriting the same motor PWM.
    if (speed == last_speed)
    {
        return;
    }

    if (speed > 0)
    {
        motor_forward(
            static_cast<float>(
                speed
            )
        );
    }
    else if (speed < 0)
    {
        motor_reverse(
            static_cast<float>(
                -speed
            )
        );
    }
    else
    {
        motor_stop();
    }

    last_speed = speed;
}


// ==========================================================
// SERVO COMMAND
// ==========================================================

void apply_servo_command(int steering)
{
    steering =
        clamp_steering(
            steering
        );

    // Don't rewrite identical servo position.
    if (steering == last_steering)
    {
        return;
    }

    servo_write(
        steering
    );

    last_steering = steering;
}


// ==========================================================
// FORCE SAFE STATE
// ==========================================================

void force_safe_state()
{
    // Always physically issue the stop.
    // Do NOT rely only on software state.

    motor_stop();

    servo_center();

    last_speed = 0;
    last_steering = STEERING_CENTER;
}


// ==========================================================
// MAIN
// ==========================================================

int main()
{
    // ======================================================
    // SERIAL
    // ======================================================

    stdio_init_all();

    sleep_ms(1500);

    printf("\n");
    printf("====================================\n");
    printf("VELOZ PICO 2\n");
    printf("I2C SLAVE CONTROLLER\n");
    printf("RASPBERRY PI 5 = MASTER\n");
    printf("====================================\n");


    // ======================================================
    // MOTOR
    // ======================================================

    printf("[1] Motor init...\n");

    motor_init();

    motor_stop();

    printf("[1] Motor ready - STOPPED\n");


    // ======================================================
    // SERVO
    // ======================================================

    printf("[2] Servo init...\n");

    servo_init();

    servo_center();

    printf("[2] Servo ready - CENTER\n");


    // ======================================================
    // ENCODER
    // ======================================================

    printf("[3] Encoder init...\n");

    encoder_init();

    printf("[3] Encoder ready\n");


    // ======================================================
    // IMU
    // ======================================================

    printf("[4] IMU init...\n");

    bool imu_ready =
        imu_init();

    if (imu_ready)
    {
        printf("[4] IMU ready\n");
    }
    else
    {
        printf(
            "[4] IMU unavailable\n"
        );

        printf(
            "[4] Continuing without IMU control\n"
        );
    }


    // ======================================================
    // PI I2C
    // ======================================================

    printf("[5] Pi I2C slave init...\n");

    pi_i2c_init();

    printf(
        "[5] I2C slave ready at 0x39\n"
    );


    // ======================================================
    // INITIAL SAFE STATE
    // ======================================================

    force_safe_state();

    const uint32_t start_time =
        to_ms_since_boot(
            get_absolute_time()
        );

    last_command_time_ms =
        start_time;

    last_telemetry_time_ms =
        start_time;

    last_debug_time_ms =
        start_time;

    watchdog_active = true;


    // ======================================================
    // READY
    // ======================================================

    printf("\n");
    printf("====================================\n");
    printf("PICO READY\n");
    printf("Waiting for Raspberry Pi commands\n");
    printf(
        "WATCHDOG: %u ms\n",
        COMMAND_TIMEOUT_MS
    );
    printf(
        "STEERING: %d / %d / %d\n",
        STEERING_MIN,
        STEERING_CENTER,
        STEERING_MAX
    );
    printf("====================================\n");


    // ======================================================
    // MAIN LOOP
    // ======================================================

    while (true)
    {
        const uint32_t now_ms =
            to_ms_since_boot(
                get_absolute_time()
            );


        // ==================================================
        // SENSOR UPDATE
        // ==================================================

        encoder_update();

        bool imu_updated = false;

        if (imu_ready)
        {
            imu_updated =
                imu_update();
        }


        // ==================================================
        // RECEIVE COMMAND FROM RASPBERRY PI
        // ==================================================

        if (
            pi_i2c_command_received()
        )
        {
            const PiCommand command =
                pi_i2c_get_command();


            // ----------------------------------------------
            // Clear command flag immediately after copying.
            // ----------------------------------------------

            pi_i2c_clear_command_flag();


            // ----------------------------------------------
            // Refresh watchdog
            // ----------------------------------------------

            last_command_time_ms =
                now_ms;

            if (watchdog_active)
            {
                watchdog_active =
                    false;

                printf(
                    "MASTER CONNECTED\n"
                );
            }


            // ==============================================
            // EMERGENCY STOP
            // ==============================================

            if (
                command.emergency_stop
            )
            {
                force_safe_state();

                printf(
                    "ESTOP -> MOTOR STOP\n"
                );
            }


            // ==============================================
            // NORMAL CONTROL
            // ==============================================

            else
            {
                // ------------------------------------------
                // SPEED
                // ------------------------------------------

                int speed =
                    static_cast<int>(
                        command.speed_percent
                    );

                if (speed > 100)
                {
                    speed = 100;
                }

                if (speed < -100)
                {
                    speed = -100;
                }


                // ------------------------------------------
                // STEERING
                // ------------------------------------------

                const int steering =
                    clamp_steering(
                        static_cast<int>(
                            command.steering_angle
                        )
                    );


                // ------------------------------------------
                // APPLY
                // ------------------------------------------

                apply_motor_command(
                    speed
                );

                apply_servo_command(
                    steering
                );
            }
        }


        // ==================================================
        // WATCHDOG
        // ==================================================

        const uint32_t command_age =
            now_ms -
            last_command_time_ms;

        if (
            command_age >
            COMMAND_TIMEOUT_MS
        )
        {
            // IMPORTANT:
            //
            // Force the hardware safe EVERY LOOP while
            // communication is missing.
            //
            // We do not trust cached software values.

            motor_stop();

            servo_center();

            last_speed = 0;
            last_steering =
                STEERING_CENTER;

            if (!watchdog_active)
            {
                watchdog_active =
                    true;

                printf(
                    "WATCHDOG TIMEOUT\n"
                );

                printf(
                    "MASTER LOST -> MOTOR STOP\n"
                );
            }
        }


        // ==================================================
        // TELEMETRY
        // ==================================================

        if (
            now_ms -
            last_telemetry_time_ms >=
            TELEMETRY_INTERVAL_MS
        )
        {
            last_telemetry_time_ms =
                now_ms;

            PicoTelemetry telemetry {};

            telemetry.encoder_count =
                encoder_get_count();

            telemetry.rpm_x10 =
                static_cast<int16_t>(
                    encoder_get_rpm()
                    * 10.0f
                );


            // ----------------------------------------------
            // IMU
            // ----------------------------------------------

            if (imu_ready)
            {
                telemetry.yaw_x10 =
                    static_cast<int16_t>(
                        imu_get_yaw_deg()
                        * 10.0f
                    );

                telemetry.pitch_x10 =
                    static_cast<int16_t>(
                        imu_get_pitch_deg()
                        * 10.0f
                    );

                telemetry.roll_x10 =
                    static_cast<int16_t>(
                        imu_get_roll_deg()
                        * 10.0f
                    );
            }
            else
            {
                telemetry.yaw_x10 = 0;
                telemetry.pitch_x10 = 0;
                telemetry.roll_x10 = 0;
            }


            pi_i2c_set_telemetry(
                telemetry
            );
        }


        // ==================================================
        // DEBUG
        // ==================================================

        if (
            now_ms -
            last_debug_time_ms >=
            DEBUG_INTERVAL_MS
        )
        {
            last_debug_time_ms =
                now_ms;

            printf(
                "PICO"
                " | Speed=%d"
                " | Steering=%d"
                " | RPM=%.1f"
                " | WD=%s"
                " | Age=%lu"
                " | IMU=%d\n",

                last_speed,

                last_steering,

                encoder_get_rpm(),

                watchdog_active
                    ? "STOP"
                    : "OK",

                static_cast<unsigned long>(
                    command_age
                ),

                imu_updated
                    ? 1
                    : 0
            );
        }


        // ==================================================
        // CONTROL LOOP TIMING
        // ==================================================

        tight_loop_contents();

        sleep_ms(
            MAIN_LOOP_MS
        );
    }


    return 0;
}