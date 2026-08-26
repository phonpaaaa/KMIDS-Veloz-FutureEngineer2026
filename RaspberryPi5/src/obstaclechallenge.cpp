#include "obstaclechallenge.h"

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
constexpr int STEERING_LEFT   = 60;
constexpr int STEERING_RIGHT  = 120;

constexpr int SPEED_NORMAL = 35;
constexpr int SPEED_TURN   = 25;

constexpr float FRONT_REACT_MM = 700.0f;
constexpr float SIDE_REACT_MM  = 400.0f;

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

void obstacle_challenge_start()
{
    if (running.load())
    {
        return;
    }

    running = true;

    std::cout
        << "\n====================================\n"
        << "MOCK OBSTACLE CHALLENGE\n"
        << "LiDAR only for now\n"
        << "====================================\n";

    while (running.load())
    {
        lidar_update();

        const LidarDistances d =
            lidar_get_distances();

        if (
            d.front_mm <= 0 ||
            d.left_mm <= 0 ||
            d.right_mm <= 0
        )
        {
            stop_robot();

            std::this_thread::sleep_for(
                std::chrono::milliseconds(LOOP_MS)
            );

            continue;
        }

        int steering = STEERING_CENTER;
        int speed = SPEED_NORMAL;

        // Something directly ahead.
        if (d.front_mm < FRONT_REACT_MM)
        {
            speed = SPEED_TURN;

            // More room on left.
            if (d.left_mm > d.right_mm)
            {
                steering = STEERING_LEFT;

                std::cout
                    << "OBSTACLE FRONT -> LEFT"
                    << std::endl;
            }
            else
            {
                steering = STEERING_RIGHT;

                std::cout
                    << "OBSTACLE FRONT -> RIGHT"
                    << std::endl;
            }
        }

        // Very close left side.
        else if (d.left_mm < SIDE_REACT_MM)
        {
            steering = STEERING_RIGHT;

            std::cout
                << "LEFT BLOCKED -> RIGHT"
                << std::endl;
        }

        // Very close right side.
        else if (d.right_mm < SIDE_REACT_MM)
        {
            steering = STEERING_LEFT;

            std::cout
                << "RIGHT BLOCKED -> LEFT"
                << std::endl;
        }

        send_command(
            speed,
            steering
        );

        std::cout
            << "OBS"
            << " | F=" << d.front_mm
            << " | L=" << d.left_mm
            << " | R=" << d.right_mm
            << " | Speed=" << speed
            << " | Servo=" << steering
            << std::endl;

        std::this_thread::sleep_for(
            std::chrono::milliseconds(LOOP_MS)
        );
    }

    stop_robot();
}

void obstacle_challenge_stop()
{
    running = false;
}

bool obstacle_challenge_is_running()
{
    return running.load();
}