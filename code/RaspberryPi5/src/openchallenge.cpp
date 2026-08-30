#include "openchallenge.h"

#include "lidar.h"
#include "navigation.h"
#include "pico_i2c.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

namespace
{

// ==========================================================
// OPEN CHALLENGE STATE
// ==========================================================

std::atomic<bool> running{false};

// ==========================================================
// CONTROL LOOP
// ==========================================================

// 20 Hz control loop
constexpr int CONTROL_LOOP_MS = 50;

// ==========================================================
// STEERING LIMITS
// ==========================================================

constexpr int STEERING_MIN = 40;
constexpr int STEERING_CENTER = 90;
constexpr int STEERING_MAX = 140;

// ==========================================================
// LIDAR WATCHDOG
// ==========================================================

// Stop the robot if no NEW LiDAR scan has arrived
// for this amount of time.
constexpr int LIDAR_TIMEOUT_MS = 500;

// ==========================================================
// SEND STOP
// ==========================================================

void send_stop_command()
{
    PicoCommand command{};

    command.speed_percent = 0;
    command.steering_angle = STEERING_CENTER;

    // IMPORTANT:
    // 1 = emergency stop
    command.emergency_stop = 1;

    pico_i2c_send_command(command);
}

} // namespace

// ==========================================================
// START OPEN CHALLENGE
// ==========================================================

void open_challenge_start()
{
    // Prevent accidental double start.
    if (running.load())
    {
        return;
    }

    running.store(true);

    // ======================================================
    // INITIALIZE NAVIGATION
    // ======================================================

    navigation_init();

    std::cout
        << "\n"
        << "====================================\n"
        << "VELOZ - WRO OPEN CHALLENGE\n"
        << "RPLIDAR S3 + PICO 2\n"
        << "====================================\n"
        << "LEFT wall  -> steer RIGHT\n"
        << "RIGHT wall -> steer LEFT\n"
        << "FRONT wall -> choose open side\n"
        << "LiDAR watchdog: "
        << LIDAR_TIMEOUT_MS
        << " ms\n"
        << "====================================\n"
        << std::endl;

    // ======================================================
    // LIDAR WATCHDOG TIMER
    // ======================================================

    auto last_lidar_scan =
        std::chrono::steady_clock::now();

    // ======================================================
    // DEBUG
    // ======================================================

    int debug_counter = 0;

    bool lidar_stale = false;

    // ======================================================
    // MAIN CONTROL LOOP
    // ======================================================

    while (running.load())
    {
        // ==================================================
        // 1. CHECK FOR NEW LIDAR SCAN
        // ==================================================

        const bool lidar_new_data =
            lidar_update();

        const auto now =
            std::chrono::steady_clock::now();

        if (lidar_new_data)
        {
            last_lidar_scan = now;

            // If LiDAR recovered after being stale,
            // allow navigation again.
            if (lidar_stale)
            {
                std::cout
                    << "OPEN | LIDAR RECOVERED"
                    << std::endl;

                lidar_stale = false;
            }
        }

        // ==================================================
        // 2. CALCULATE LIDAR AGE
        // ==================================================

        const auto lidar_age_ms =
            std::chrono::duration_cast<
                std::chrono::milliseconds
            >(
                now - last_lidar_scan
            ).count();

        // ==================================================
        // 3. LIDAR WATCHDOG
        // ==================================================

        if (
            lidar_age_ms >
            LIDAR_TIMEOUT_MS
        )
        {
            // ----------------------------------------------
            // NO FRESH LIDAR DATA
            //
            // STOP.
            //
            // Do not navigate using old measurements.
            // ----------------------------------------------

            send_stop_command();

            if (!lidar_stale)
            {
                std::cerr
                    << "OPEN | LIDAR STALE "
                    << lidar_age_ms
                    << " ms -> STOP"
                    << std::endl;

                lidar_stale = true;
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    CONTROL_LOOP_MS
                )
            );

            continue;
        }

        // ==================================================
        // 4. READ PICO TELEMETRY
        // ==================================================

        PicoTelemetry telemetry{};

        float yaw_deg = 0.0f;

        const bool telemetry_ok =
            pico_i2c_read_telemetry(
                telemetry
            );

        if (telemetry_ok)
        {
            yaw_deg =
                static_cast<float>(
                    telemetry.yaw_x10
                )
                / 10.0f;
        }

        // ==================================================
        // 5. NAVIGATION
        // ==================================================

        const NavigationCommand nav =
            navigation_update(
                yaw_deg
            );

        // ==================================================
        // 6. BUILD PICO COMMAND
        // ==================================================

        PicoCommand command{};

        command.speed_percent =
            static_cast<std::int8_t>(
                std::clamp(
                    nav.speed_percent,
                    -100,
                    100
                )
            );

        command.steering_angle =
            static_cast<std::uint8_t>(
                std::clamp(
                    nav.steering_angle,
                    STEERING_MIN,
                    STEERING_MAX
                )
            );

        command.emergency_stop =
            nav.emergency_stop
                ? 1
                : 0;

        // ==================================================
        // 7. SEND COMMAND TO PICO
        // ==================================================

        const bool command_sent =
            pico_i2c_send_command(
                command
            );

        // ==================================================
        // 8. I2C FAILURE
        // ==================================================

        if (!command_sent)
        {
            std::cerr
                << "OPEN | PICO COMMAND FAILED"
                << std::endl;

            // Attempt immediate stop.
            send_stop_command();

            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    CONTROL_LOOP_MS
                )
            );

            continue;
        }

        // ==================================================
        // 9. DEBUG OUTPUT
        // ==================================================

        ++debug_counter;

        if (debug_counter >= 5)
        {
            debug_counter = 0;

            const LidarDistances d =
                lidar_get_distances();

            std::cout
                << "OPEN"

                << " | F="
                << d.front_mm

                << " | L="
                << d.left_mm

                << " | R="
                << d.right_mm

                << " | B="
                << d.back_mm

                << " | Yaw="
                << yaw_deg

                << " | Speed="
                << static_cast<int>(
                    command.speed_percent
                )

                << " | Servo="
                << static_cast<int>(
                    command.steering_angle
                )

                << " | LiDAR="
                << (
                    lidar_new_data
                        ? "NEW"
                        : "WAIT"
                )

                << " | Age="
                << lidar_age_ms
                << "ms"

                << " | Pico="
                << (
                    telemetry_ok
                        ? "OK"
                        : "NO DATA"
                )

                << std::endl;
        }

        // ==================================================
        // 10. CONTROL LOOP DELAY
        // ==================================================

        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                CONTROL_LOOP_MS
            )
        );
    }

    // ======================================================
    // CHALLENGE STOPPED
    // ======================================================

    send_stop_command();

    std::cout
        << "\nVELOZ OPEN CHALLENGE STOPPED"
        << std::endl;
}

// ==========================================================
// STOP OPEN CHALLENGE
// ==========================================================

void open_challenge_stop()
{
    running.store(false);
}

// ==========================================================
// CHECK RUNNING STATE
// ==========================================================

bool open_challenge_is_running()
{
    return running.load();
}