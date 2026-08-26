#include "lidar.h"
#include "openchallenge.h"
#include "obstaclechallenge.h"
#include "parking.h"
#include "pico_i2c.h"

#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <thread>

// ==========================================================
// VELOZ
// WRO FUTURE ENGINEER
// ==========================================================

namespace
{

// ==========================================================
// PROGRAM MODE
// ==========================================================
//
// Change ONLY this line:
//
// OPEN_CHALLENGE
// OBSTACLE_CHALLENGE
// PARKING
//
// ==========================================================

enum class RobotMode
{
    OPEN_CHALLENGE,
    OBSTACLE_CHALLENGE,
    PARKING
};

constexpr RobotMode ROBOT_MODE =
    RobotMode::OPEN_CHALLENGE;

// ==========================================================
// GLOBAL STOP
// ==========================================================

volatile std::sig_atomic_t stop_requested = 0;

// ==========================================================
// SIGNAL HANDLER
// ==========================================================

void signal_handler(int)
{
    stop_requested = 1;

    // Tell every possible challenge loop to stop.
    open_challenge_stop();
    obstacle_challenge_stop();
    parking_stop();
}

// ==========================================================
// SAFE STOP
// ==========================================================

void send_stop()
{
    PicoCommand command{};

    command.speed_percent = 0;
    command.steering_angle = 90;
    command.emergency_stop = 1;

    pico_i2c_send_command(command);
}

// ==========================================================
// MODE NAME
// ==========================================================

const char* mode_name()
{
    switch (ROBOT_MODE)
    {
        case RobotMode::OPEN_CHALLENGE:
            return "OPEN CHALLENGE";

        case RobotMode::OBSTACLE_CHALLENGE:
            return "OBSTACLE CHALLENGE";

        case RobotMode::PARKING:
            return "PARKING";
    }

    return "UNKNOWN";
}

// ==========================================================
// START SELECTED MODE
// ==========================================================

void start_selected_mode()
{
    switch (ROBOT_MODE)
    {
        case RobotMode::OPEN_CHALLENGE:
        {
            open_challenge_start();
            break;
        }

        case RobotMode::OBSTACLE_CHALLENGE:
        {
            obstacle_challenge_start();
            break;
        }

        case RobotMode::PARKING:
        {
            parking_start();
            break;
        }
    }
}

} // namespace

// ==========================================================
// MAIN
// ==========================================================

int main()
{
    // ======================================================
    // SIGNALS
    // ======================================================

    std::signal(
        SIGINT,
        signal_handler
    );

    std::signal(
        SIGTERM,
        signal_handler
    );

    // ======================================================
    // STARTUP MESSAGE
    // ======================================================

    std::cout
        << "\n"
        << "====================================\n"
        << "VELOZ\n"
        << "WRO FUTURE ENGINEER\n"
        << "====================================\n"
        << "MODE: "
        << mode_name()
        << "\n"
        << "====================================\n"
        << std::endl;

    // ======================================================
    // 1. INITIALIZE PICO 2
    // ======================================================

    std::cout
        << "[1] Connecting to Pico 2..."
        << std::endl;

    if (!pico_i2c_init())
    {
        std::cerr
            << "ERROR: Pico 2 initialization failed."
            << std::endl;

        return 1;
    }

    // ======================================================
    // IMMEDIATE MOTOR STOP
    // ======================================================

    send_stop();

    std::this_thread::sleep_for(
        std::chrono::milliseconds(100)
    );

    std::cout
        << "[1] Pico ready"
        << std::endl;

    // ======================================================
    // 2. INITIALIZE RPLIDAR S3
    // ======================================================

    std::cout
        << "[2] Initializing RPLIDAR S3..."
        << std::endl;

    if (!lidar_init())
    {
        std::cerr
            << "ERROR: RPLIDAR S3 initialization failed."
            << std::endl;

        send_stop();

        return 1;
    }

    std::cout
        << "[2] RPLIDAR S3 ready"
        << std::endl;

    // ======================================================
    // 3. WAIT FOR REAL LIDAR DATA
    // ======================================================

    std::cout
        << "[3] Waiting for LiDAR data..."
        << std::endl;

    bool lidar_data_received = false;

    const auto lidar_wait_start =
        std::chrono::steady_clock::now();

    while (!stop_requested)
    {
        if (lidar_update())
        {
            const LidarDistances d =
                lidar_get_distances();

            // At least one important direction
            // must contain a valid measurement.
            if (
                d.front_mm > 0.0f ||
                d.left_mm  > 0.0f ||
                d.right_mm > 0.0f
            )
            {
                lidar_data_received = true;

                break;
            }
        }

        const auto now =
            std::chrono::steady_clock::now();

        const auto elapsed_ms =
            std::chrono::duration_cast<
                std::chrono::milliseconds
            >(
                now - lidar_wait_start
            ).count();

        // Don't wait forever.
        if (elapsed_ms > 5000)
        {
            break;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(20)
        );
    }

    // ======================================================
    // LIDAR STARTUP FAILURE
    // ======================================================

    if (!lidar_data_received)
    {
        std::cerr
            << "ERROR: No valid LiDAR data received."
            << std::endl;

        send_stop();

        lidar_close();

        return 1;
    }

    // ======================================================
    // SHOW INITIAL DISTANCES
    // ======================================================

    const LidarDistances initial =
        lidar_get_distances();

    std::cout
        << "[3] LiDAR data confirmed"
        << std::endl;

    std::cout
        << "    F="
        << initial.front_mm

        << " | L="
        << initial.left_mm

        << " | R="
        << initial.right_mm

        << " | B="
        << initial.back_mm

        << std::endl;

    // ======================================================
    // 4. FINAL SAFETY STOP BEFORE START
    // ======================================================

    send_stop();

    std::this_thread::sleep_for(
        std::chrono::milliseconds(300)
    );

    // ======================================================
    // 5. START SELECTED PROGRAM
    // ======================================================

    if (!stop_requested)
    {
        std::cout
            << "\n"
            << "====================================\n"
            << "ROBOT READY\n"
            << "====================================\n"
            << "STARTING: "
            << mode_name()
            << "\n"
            << "Ctrl+C -> STOP\n"
            << "====================================\n"
            << std::endl;

        start_selected_mode();
    }

    // ======================================================
    // 6. SHUTDOWN
    // ======================================================

    std::cout
        << "\n"
        << "STOPPING VELOZ..."
        << std::endl;

    // Stop motor.
    send_stop();

    std::this_thread::sleep_for(
        std::chrono::milliseconds(100)
    );

    // Close S3.
    lidar_close();

    // Final motor stop.
    send_stop();

    std::cout
        << "\n"
        << "====================================\n"
        << "VELOZ SHUTDOWN COMPLETE\n"
        << "====================================\n"
        << std::endl;

    return 0;
}