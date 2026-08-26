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

// ==========================================================
// RUN STATE
// ==========================================================

std::atomic<bool> running{false};

// ==========================================================
// LOOP
// ==========================================================

constexpr int CONTROL_LOOP_MS = 50;

// ==========================================================
// MOTOR SPEED
// ==========================================================

constexpr int SPEED_NORMAL   = 40;
constexpr int SPEED_OBSTACLE = 30;
constexpr int SPEED_CORRECT  = 35;

// ==========================================================
// STEERING
//
// 40  = LEFT
// 90  = CENTER
// 140 = RIGHT
// ==========================================================

constexpr int STEERING_MIN    = 40;
constexpr int STEERING_CENTER = 90;
constexpr int STEERING_MAX    = 140;

constexpr int STEERING_LEFT  = 60;
constexpr int STEERING_RIGHT = 120;

// ==========================================================
// LIDAR
// ==========================================================

constexpr float MIN_VALID_MM = 80.0f;
constexpr float MAX_VALID_MM = 6000.0f;

constexpr float FRONT_DANGER_MM = 450.0f;
constexpr float SIDE_DANGER_MM  = 220.0f;

// ==========================================================
// CAMERA
// ==========================================================

// Ignore tiny red/green noise.
constexpr int MIN_OBSTACLE_AREA = 700;

// Horizontal frame regions.
//
// Example for 640 px:
//
// LEFT        CENTER        RIGHT
// 0 --- 220 --- 420 --- 640
//
constexpr int FRAME_LEFT_LIMIT  = 220;
constexpr int FRAME_RIGHT_LIMIT = 420;

// ==========================================================
// HELPERS
// ==========================================================

bool valid_distance(float value)
{
    return
        value >= MIN_VALID_MM &&
        value <= MAX_VALID_MM;
}

void send_command(
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

    pico_i2c_send_command(command);
}

void stop_robot()
{
    send_command(
        0,
        STEERING_CENTER,
        true
    );
}

} // namespace

// ==========================================================
// START
// ==========================================================

void obstacle_challenge_start()
{
    if (running.load())
    {
        return;
    }

    running = true;

    std::cout
        << "\n"
        << "====================================\n"
        << "VELOZ - OBSTACLE CHALLENGE\n"
        << "CAMERA + RPLIDAR S3 + PICO 2\n"
        << "====================================\n"
        << std::endl;

    // ======================================================
    // CAMERA INIT
    // ======================================================

    std::cout
        << "[CAMERA] Initializing..."
        << std::endl;

    if (!camera_init())
    {
        std::cerr
            << "[CAMERA] INITIALIZATION FAILED"
            << std::endl;

        stop_robot();
        running = false;
        return;
    }

    std::cout
        << "[CAMERA] READY"
        << std::endl;

    // ======================================================
    // LIDAR WATCHDOG
    // ======================================================

    auto last_lidar_scan =
        std::chrono::steady_clock::now();

    constexpr int LIDAR_TIMEOUT_MS = 500;

    bool lidar_received = false;

    // ======================================================
    // DEBUG
    // ======================================================

    int debug_counter = 0;

    // ======================================================
    // MAIN LOOP
    // ======================================================

    while (running.load())
    {
        // ==================================================
        // 1. LIDAR
        // ==================================================

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

        // ==================================================
        // 2. CAMERA
        // ==================================================

        const bool new_camera =
            camera_update();

        CameraDetection camera{};

        if (new_camera)
        {
            camera =
                camera_get_detection();
        }

        // ==================================================
        // 3. DEFAULT COMMAND
        // ==================================================

        int speed =
            SPEED_NORMAL;

        int steering =
            STEERING_CENTER;

        const char* action =
            "STRAIGHT";

        // ==================================================
        // 4. RED OBSTACLE
        // ==================================================
        //
        // MOCK RULE:
        //
        // RED -> steer LEFT around obstacle.
        //
        // We can reverse this later if needed.
        // ==================================================

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

            // If red object is already far left,
            // reduce steering.
            if (
                camera.red_x <
                FRAME_LEFT_LIMIT
            )
            {
                steering =
                    STEERING_CENTER;
            }
        }

        // ==================================================
        // 5. GREEN OBSTACLE
        // ==================================================
        //
        // MOCK RULE:
        //
        // GREEN -> steer RIGHT around obstacle.
        // ==================================================

        else if (green_valid)
        {
            speed =
                SPEED_OBSTACLE;

            steering =
                STEERING_RIGHT;

            action =
                "GREEN -> RIGHT";

            // If green object is already far right,
            // reduce steering.
            if (
                camera.green_x >
                FRAME_RIGHT_LIMIT
            )
            {
                steering =
                    STEERING_CENTER;
            }
        }

        // ==================================================
        // 6. NO COLORED OBSTACLE
        //
        // Use S3 for basic wall correction.
        // ==================================================

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

            // LEFT wall very close
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

            // RIGHT wall very close
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

        // ==================================================
        // 7. FRONT COLLISION PROTECTION
        // ==================================================

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

            // Choose whichever side has more room.
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

        // ==================================================
        // 8. SEND TO PICO
        // ==================================================

        send_command(
            speed,
            steering,
            false
        );

        // ==================================================
        // 9. DEBUG
        // ==================================================

        ++debug_counter;

        if (debug_counter >= 5)
        {
            debug_counter = 0;

            std::cout
                << "OBS"

                << " | F="
                << d.front_mm

                << " | L="
                << d.left_mm

                << " | R="
                << d.right_mm

                << " | RED="
                << (
                    red_valid
                        ? "YES"
                        : "NO"
                );

            if (red_valid)
            {
                std::cout
                    << "("
                    << camera.red_x
                    << ", area="
                    << camera.red_area
                    << ")";
            }

            std::cout
                << " | GREEN="
                << (
                    green_valid
                        ? "YES"
                        : "NO"
                );

            if (green_valid)
            {
                std::cout
                    << "("
                    << camera.green_x
                    << ", area="
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

        // ==================================================
        // LOOP DELAY
        // ==================================================

        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                CONTROL_LOOP_MS
            )
        );
    }

    // ======================================================
    // SHUTDOWN
    // ======================================================

    stop_robot();

    camera_close();

    std::cout
        << "\n"
        << "VELOZ OBSTACLE CHALLENGE STOPPED"
        << std::endl;
}

// ==========================================================
// STOP
// ==========================================================

void obstacle_challenge_stop()
{
    running = false;
}

// ==========================================================
// RUNNING
// ==========================================================

bool obstacle_challenge_is_running()
{
    return running.load();
}