#include "parking.h"

#include "lidar.h"
#include "pico_i2c.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

namespace
{

std::atomic<bool> running{false};

constexpr int LOOP_MS = 50;

constexpr int STEERING_CENTER = 90;
constexpr int STEERING_LEFT   = 55;
constexpr int STEERING_RIGHT  = 125;

constexpr int SPEED_FORWARD = 20;
constexpr int SPEED_REVERSE = -18;

// Mock thresholds.
constexpr float PARK_FRONT_MM = 350.0f;
constexpr float PARK_SIDE_MM  = 300.0f;
constexpr float PARK_BACK_MM  = 250.0f;

enum class ParkingState
{
    SEARCHING,
    ALIGN,
    REVERSING,
    STRAIGHTEN,
    DONE
};

ParkingState state =
    ParkingState::SEARCHING;

void send_command(
    int speed,
    int steering
)
{
    PicoCommand command{};

    command.speed_percent =
        static_cast<std::int8_t>(
            std::clamp(speed, -100, 100)
        );

    command.steering_angle =
        static_cast<std::uint8_t>(
            std::clamp(steering, 40, 140)
        );

    command.emergency_stop = 0;

    pico_i2c_send_command(command);
}

void stop_robot()
{
    send_command(
        0,
        STEERING_CENTER
    );
}

}

void parking_start()
{
    if (running.load())
    {
        return;
    }

    running = true;
    state = ParkingState::SEARCHING;

    std::cout
        << "\n====================================\n"
        << "MOCK PARKING PROGRAM\n"
        << "====================================\n";

    while (running.load())
    {
        lidar_update();

        const LidarDistances d =
            lidar_get_distances();

        switch (state)
        {
            // ------------------------------------------------
            // Find a rough parking position.
            // ------------------------------------------------
            case ParkingState::SEARCHING:
            {
                send_command(
                    SPEED_FORWARD,
                    STEERING_CENTER
                );

                if (
                    d.front_mm > 0 &&
                    d.front_mm < PARK_FRONT_MM
                )
                {
                    state =
                        ParkingState::ALIGN;

                    std::cout
                        << "PARK | ALIGN"
                        << std::endl;
                }

                break;
            }

            // ------------------------------------------------
            // Turn car before reversing.
            // ------------------------------------------------
            case ParkingState::ALIGN:
            {
                send_command(
                    SPEED_FORWARD,
                    STEERING_LEFT
                );

                std::this_thread::sleep_for(
                    std::chrono::milliseconds(600)
                );

                state =
                    ParkingState::REVERSING;

                break;
            }

            // ------------------------------------------------
            // Reverse into space.
            // ------------------------------------------------
            case ParkingState::REVERSING:
            {
                send_command(
                    SPEED_REVERSE,
                    STEERING_RIGHT
                );

                if (
                    d.back_mm > 0 &&
                    d.back_mm < PARK_BACK_MM
                )
                {
                    state =
                        ParkingState::STRAIGHTEN;

                    std::cout
                        << "PARK | STRAIGHTEN"
                        << std::endl;
                }

                break;
            }

            // ------------------------------------------------
            // Center the wheels and finish.
            // ------------------------------------------------
            case ParkingState::STRAIGHTEN:
            {
                send_command(
                    SPEED_REVERSE,
                    STEERING_CENTER
                );

                std::this_thread::sleep_for(
                    std::chrono::milliseconds(400)
                );

                state =
                    ParkingState::DONE;

                break;
            }

            // ------------------------------------------------
            // Parked.
            // ------------------------------------------------
            case ParkingState::DONE:
            {
                stop_robot();

                std::cout
                    << "PARK | DONE"
                    << std::endl;

                running = false;

                break;
            }
        }

        std::cout
            << "PARK"
            << " | F=" << d.front_mm
            << " | L=" << d.left_mm
            << " | R=" << d.right_mm
            << " | B=" << d.back_mm
            << std::endl;

        std::this_thread::sleep_for(
            std::chrono::milliseconds(LOOP_MS)
        );
    }

    stop_robot();
}

void parking_stop()
{
    running = false;
}

bool parking_is_running()
{
    return running.load();
}