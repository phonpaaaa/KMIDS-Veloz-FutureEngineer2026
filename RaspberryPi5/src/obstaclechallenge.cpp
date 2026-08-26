#include "obstaclechallenge.h"

#include "camera.h"
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

constexpr int CONTROL_LOOP_MS = 50;

constexpr int SPEED_NORMAL = 40;
constexpr int SPEED_OBSTACLE = 30;
constexpr int SPEED_CORRECT = 35;

constexpr int STEERING_MIN = 40;
constexpr int STEERING_CENTER = 90;
constexpr int STEERING_MAX = 140;

constexpr int STEERING_LEFT = 60;
constexpr int STEERING_RIGHT = 120;

constexpr float MIN_VALID_MM = 80.0f;
constexpr float MAX_VALID_MM = 6000.0f;

constexpr float FRONT_DANGER_MM = 450.0f;
constexpr float SIDE_DANGER_MM = 220.0f;

constexpr int MIN_OBSTACLE_AREA = 700;

constexpr int FRAME_LEFT_LIMIT = 220;
constexpr int FRAME_RIGHT_LIMIT = 420;

bool valid_distance(float value)
{
    return
        value >= MIN_VALID_MM &&
        value <= MAX_VALID_MM;
}

bool send_command(
    int speed,
    int steering,
    bool emergency_stop = false
)
{
    PicoCommand command{};

    command.speed_percent =
        static_cast<std::int8_t>(
            std::clamp(
                speed,
                -100,
                100
            )
        );

    command.steering_angle =
        static_cast<std::uint8_t>(
            std::clamp(
                steering,
                STEERING_MIN,
                STEERING_MAX
            )
        );

    command.emergency_stop =
        emergency_stop ? 1 : 0;

    return pico_i2c_send_command(command);
}

void stop_robot()
{
    send_command(
        0,
        STEERING_CENTER,
        true
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
        << "VELOZ - OBSTACLE CHALLENGE\n"
        << "CAMERA + RPLIDAR S3 + PICO 2\n"
        << "====================================\n"
        << std::endl;

    if (!camera_init())
    {
        std::cerr
            << "CAMERA INITIALIZATION FAILED"
            << std::endl;

        stop_robot();

        running = false;

        return;
    }

    std::cout
        << "CAMERA READY"
        << std::endl;

    auto last_lidar_scan =
        std::chrono::steady_clock::now();

    constexpr int LIDAR_TIMEOUT_MS = 500;

    bool lidar_received = false;

    int debug_counter = 0;

    while (running.load())
    {
        const bool new_lidar =
            lidar_update();

        const auto now =
            std::chrono::steady_clock::now();

        if (new_lidar)
        {
            last_lidar_scan = now;
            lidar_received = true;
        }

        const auto lidar_age =
            std::chrono::duration_cast<
                std::chrono::milliseconds
            >(
                now - last_lidar_scan
            ).count();

        if (
            !lidar_received ||
            lidar_age > LIDAR_TIMEOUT_MS
        )
        {
            stop_robot();

            std::cerr
                << "OBS | LIDAR STALE -> STOP"
                << std::endl;

            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    CONTROL_LOOP_MS
                )
            );

            continue;
        }

        const LidarDistances d =
            lidar_get_distances();

        camera_update();

        const CameraDetection camera =
            camera_get_detection();

        int speed =
            SPEED_NORMAL;

        int steering =
            STEERING_CENTER;

        const char* action =
            "STRAIGHT";

        const bool red_valid =
            camera.red_detected &&
            camera.red_area >=
                MIN_OBSTACLE_AREA;

        const bool green_valid =
            camera.green_detected &&
            camera.green_area >=
                MIN_OBSTACLE_AREA;

        if (red_valid)
        {
            speed =
                SPEED_OBSTACLE;

            steering =
                STEERING_LEFT;

            action =
                "RED -> LEFT";

            if (
                camera.red_x <
                FRAME_LEFT_LIMIT
            )
            {
                steering =
                    STEERING_CENTER;
            }
        }
        else if (green_valid)
        {
            speed =
                SPEED_OBSTACLE;

            steering =
                STEERING_RIGHT;

            action =
                "GREEN -> RIGHT";

            if (
                camera.green_x >
                FRAME_RIGHT_LIMIT
            )
            {
                steering =
                    STEERING_CENTER;
            }
        }
        else
        {
            const bool left_valid =
                valid_distance(
                    d.left_mm
                );

            const bool right_valid =
                valid_distance(
                    d.right_mm
                );

            if (
                left_valid &&
                d.left_mm <
                    SIDE_DANGER_MM
            )
            {
                speed =
                    SPEED_CORRECT;

                steering =
                    STEERING_RIGHT;

                action =
                    "LEFT WALL -> RIGHT";
            }
            else if (
                right_valid &&
                d.right_mm <
                    SIDE_DANGER_MM
            )
            {
                speed =
                    SPEED_CORRECT;

                steering =
                    STEERING_LEFT;

                action =
                    "RIGHT WALL -> LEFT";
            }
        }

        if (
            valid_distance(
                d.front_mm
            ) &&
            d.front_mm <
                FRONT_DANGER_MM
        )
        {
            speed =
                SPEED_OBSTACLE;

            if (
                valid_distance(
                    d.left_mm
                ) &&
                valid_distance(
                    d.right_mm
                )
            )
            {
                if (
                    d.left_mm >
                    d.right_mm
                )
                {
                    steering =
                        STEERING_LEFT;

                    action =
                        "FRONT -> LEFT";
                }
                else
                {
                    steering =
                        STEERING_RIGHT;

                    action =
                        "FRONT -> RIGHT";
                }
            }
        }

        if (
            !send_command(
                speed,
                steering,
                false
            )
        )
        {
            std::cerr
                << "OBS | PICO COMMAND FAILED"
                << std::endl;

            stop_robot();
        }

        ++debug_counter;

        if (debug_counter >= 5)
        {
            debug_counter = 0;

            std::cout
                << "OBS"
                << " | F=" << d.front_mm
                << " | L=" << d.left_mm
                << " | R=" << d.right_mm
                << " | B=" << d.back_mm
                << " | RED="
                << (red_valid ? "YES" : "NO");

            if (red_valid)
            {
                std::cout
                    << "("
                    << camera.red_x
                    << ","
                    << camera.red_area
                    << ")";
            }

            std::cout
                << " | GREEN="
                << (green_valid ? "YES" : "NO");

            if (green_valid)
            {
                std::cout
                    << "("
                    << camera.green_x
                    << ","
                    << camera.green_area
                    << ")";
            }

            std::cout
                << " | Speed="
                << speed
                << " | Servo="
                << steering
                << " | "
                << action
                << std::endl;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                CONTROL_LOOP_MS
            )
        );
    }

    stop_robot();

    camera_close();

    std::cout
        << "VELOZ OBSTACLE CHALLENGE STOPPED"
        << std::endl;
}

void obstacle_challenge_stop()
{
    running = false;
}

bool obstacle_challenge_is_running()
{
    return running.load();
}