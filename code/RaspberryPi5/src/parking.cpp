#include "parking.h"

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

constexpr int SPEED_SEARCH = 25;
constexpr int SPEED_PARK = 18;
constexpr int SPEED_REVERSE = -18;

constexpr int STEERING_MIN = 40;
constexpr int STEERING_CENTER = 90;
constexpr int STEERING_MAX = 140;

constexpr int STEERING_LEFT = 55;
constexpr int STEERING_RIGHT = 125;

constexpr float MIN_VALID_MM = 80.0f;
constexpr float MAX_VALID_MM = 6000.0f;

constexpr float FRONT_STOP_MM = 300.0f;
constexpr float BACK_STOP_MM = 250.0f;
constexpr float SIDE_CLOSE_MM = 250.0f;

constexpr int MIN_MARKER_AREA = 700;

enum class ParkingState
{
    SEARCHING,
    ALIGNING,
    REVERSING,
    STRAIGHTENING,
    CENTERING,
    DONE
};

ParkingState state =
    ParkingState::SEARCHING;

bool valid_distance(float distance)
{
    return
        distance >= MIN_VALID_MM &&
        distance <= MAX_VALID_MM;
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

void parking_start()
{
    if (running.load())
    {
        return;
    }

    running = true;

    state =
        ParkingState::SEARCHING;

    std::cout
        << "\n====================================\n"
        << "VELOZ - PARKING\n"
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
                << "PARK | LIDAR STALE -> STOP"
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

        const bool red_visible =
            camera.red_detected &&
            camera.red_area >=
                MIN_MARKER_AREA;

        const bool green_visible =
            camera.green_detected &&
            camera.green_area >=
                MIN_MARKER_AREA;

        int speed = 0;

        int steering =
            STEERING_CENTER;

        const char* action =
            "WAIT";

        switch (state)
        {
            case ParkingState::SEARCHING:
            {
                speed =
                    SPEED_SEARCH;

                steering =
                    STEERING_CENTER;

                action =
                    "SEARCHING";

                if (
                    red_visible ||
                    green_visible
                )
                {
                    state =
                        ParkingState::ALIGNING;
                }

                break;
            }

            case ParkingState::ALIGNING:
            {
                speed =
                    SPEED_PARK;

                if (
                    valid_distance(d.left_mm) &&
                    valid_distance(d.right_mm)
                )
                {
                    if (
                        d.left_mm >
                        d.right_mm
                    )
                    {
                        steering =
                            STEERING_LEFT;
                    }
                    else
                    {
                        steering =
                            STEERING_RIGHT;
                    }
                }
                else
                {
                    steering =
                        STEERING_LEFT;
                }

                action =
                    "ALIGNING";

                if (
                    valid_distance(d.front_mm) &&
                    d.front_mm <
                        FRONT_STOP_MM
                )
                {
                    state =
                        ParkingState::REVERSING;
                }

                break;
            }

            case ParkingState::REVERSING:
            {
                speed =
                    SPEED_REVERSE;

                if (
                    valid_distance(d.left_mm) &&
                    d.left_mm <
                        SIDE_CLOSE_MM
                )
                {
                    steering =
                        STEERING_RIGHT;
                }
                else if (
                    valid_distance(d.right_mm) &&
                    d.right_mm <
                        SIDE_CLOSE_MM
                )
                {
                    steering =
                        STEERING_LEFT;
                }
                else
                {
                    steering =
                        STEERING_RIGHT;
                }

                action =
                    "REVERSING";

                if (
                    valid_distance(d.back_mm) &&
                    d.back_mm <
                        BACK_STOP_MM
                )
                {
                    state =
                        ParkingState::STRAIGHTENING;
                }

                break;
            }

            case ParkingState::STRAIGHTENING:
            {
                speed =
                    SPEED_REVERSE;

                steering =
                    STEERING_CENTER;

                action =
                    "STRAIGHTENING";

                if (
                    valid_distance(d.left_mm) &&
                    valid_distance(d.right_mm)
                )
                {
                    const float difference =
                        d.left_mm -
                        d.right_mm;

                    if (difference > 80.0f)
                    {
                        steering =
                            STEERING_LEFT;
                    }
                    else if (difference < -80.0f)
                    {
                        steering =
                            STEERING_RIGHT;
                    }
                    else
                    {
                        state =
                            ParkingState::CENTERING;
                    }
                }
                else
                {
                    state =
                        ParkingState::CENTERING;
                }

                break;
            }

            case ParkingState::CENTERING:
            {
                speed = 0;

                steering =
                    STEERING_CENTER;

                action =
                    "CENTERING";

                state =
                    ParkingState::DONE;

                break;
            }

            case ParkingState::DONE:
            {
                speed = 0;

                steering =
                    STEERING_CENTER;

                action =
                    "PARKED";

                stop_robot();

                running = false;

                break;
            }
        }

        if (running.load())
        {
            if (
                !send_command(
                    speed,
                    steering,
                    false
                )
            )
            {
                std::cerr
                    << "PARK | PICO COMMAND FAILED"
                    << std::endl;

                stop_robot();
            }
        }

        ++debug_counter;

        if (debug_counter >= 5)
        {
            debug_counter = 0;

            std::cout
                << "PARK"
                << " | F=" << d.front_mm
                << " | L=" << d.left_mm
                << " | R=" << d.right_mm
                << " | B=" << d.back_mm
                << " | RED="
                << (red_visible ? "YES" : "NO")
                << " | GREEN="
                << (green_visible ? "YES" : "NO")
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
        << "VELOZ PARKING STOPPED"
        << std::endl;
}

void parking_stop()
{
    running = false;
}

bool parking_is_running()
{
    return running.load();
}