#include "navigation.h"
#include "lidar.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace
{

// ==========================================================
// MOTOR
// ==========================================================

constexpr int SPEED_NORMAL = 72;
constexpr int SPEED_CORRECTION = 72;
constexpr int SPEED_TURN = 71;

// ==========================================================
// STEERING
//
// YOUR ROBOT:
//
// 40  = LEFT
// 90  = CENTER
// 140 = RIGHT
// ==========================================================

constexpr int STEERING_MIN = 40;
constexpr int STEERING_CENTER = 90;
constexpr int STEERING_MAX = 140;

// Normal wall correction limits.
// Do not use full steering for tiny wall corrections.
constexpr int STEERING_NORMAL_LEFT = 65;
constexpr int STEERING_NORMAL_RIGHT = 115;

// Corner steering.
constexpr int STEERING_TURN_LEFT = 60;
constexpr int STEERING_TURN_RIGHT = 120;

// ==========================================================
// LIDAR
// ==========================================================

constexpr float MIN_VALID_MM = 100.0f;
constexpr float MAX_VALID_MM = 8500.0f;

// ==========================================================
// WALL AVOIDANCE
// ==========================================================

// Start reacting to a side wall below this distance.
constexpr float SIDE_REACT_MM = 600.0f;

// Below this distance, correction becomes strong.
constexpr float SIDE_DANGER_MM = 190.0f;

// Ignore tiny differences.
constexpr float SIDE_DEADZONE_MM = 20.0f;

// ==========================================================
// FRONT / CORNER
// ==========================================================

// If something is directly in front, perform a stronger turn.
constexpr float FRONT_TURN_MM = 530.0f;

// ==========================================================
// SMOOTHING
// ==========================================================

// Servo may move at most this many degrees per update.
constexpr int MAX_STEERING_STEP = 4;

// ==========================================================
// STATE
// ==========================================================

int previous_steering = STEERING_CENTER;

// ==========================================================
// HELPERS
// ==========================================================

bool valid_distance(float distance)
{
    return
        distance >= MIN_VALID_MM &&
        distance <= MAX_VALID_MM;
}

int clamp_steering(int angle)
{
    return std::clamp(
        angle,
        STEERING_MIN,
        STEERING_MAX
    );
}

int smooth_steering(
    int current,
    int target
)
{
    target = clamp_steering(target);

    if (target > current + MAX_STEERING_STEP)
    {
        return current + MAX_STEERING_STEP;
    }

    if (target < current - MAX_STEERING_STEP)
    {
        return current - MAX_STEERING_STEP;
    }

    return target;
}

// ==========================================================
// SIDE WALL CORRECTION
// ==========================================================

int calculate_side_steering(
    const LidarDistances& d
)
{
    const bool left_valid =
        valid_distance(d.left_mm);

    const bool right_valid =
        valid_distance(d.right_mm);

    float correction = 0.0f;

    // ======================================================
    // BOTH SIDES
    // ======================================================

    if (left_valid && right_valid)
    {
        // ----------------------------------------------
        // LEFT WALL CLOSE
        //
        // Wall on LEFT
        // -> steer RIGHT
        // -> angle ABOVE 90
        // ----------------------------------------------

        if (
            d.left_mm < SIDE_REACT_MM &&
            d.left_mm < d.right_mm
        )
        {
            float error =
                SIDE_REACT_MM -
                d.left_mm;

            if (error > SIDE_DEADZONE_MM)
            {
                correction =
                    error * 0.08f;
            }
        }

        // ----------------------------------------------
        // RIGHT WALL CLOSE
        //
        // Wall on RIGHT
        // -> steer LEFT
        // -> angle BELOW 90
        // ----------------------------------------------

        else if (
            d.right_mm < SIDE_REACT_MM &&
            d.right_mm < d.left_mm
        )
        {
            float error =
                SIDE_REACT_MM -
                d.right_mm;

            if (error > SIDE_DEADZONE_MM)
            {
                correction =
                    -(error * 0.08f);
            }
        }
    }

    // ======================================================
    // LEFT ONLY
    // ======================================================

    else if (
        left_valid &&
        d.left_mm < SIDE_REACT_MM
    )
    {
        float error =
            SIDE_REACT_MM -
            d.left_mm;

        if (error > SIDE_DEADZONE_MM)
        {
            correction =
                error * 0.08f;
        }
    }

    // ======================================================
    // RIGHT ONLY
    // ======================================================

    else if (
        right_valid &&
        d.right_mm < SIDE_REACT_MM
    )
    {
        float error =
            SIDE_REACT_MM -
            d.right_mm;

        if (error > SIDE_DEADZONE_MM)
        {
            correction =
                -(error * 0.08f);
        }
    }

    correction =
        std::clamp(
            correction,
            -25.0f,
            25.0f
        );

    int target =
        STEERING_CENTER +
        static_cast<int>(
            std::round(correction)
        );

    return std::clamp(
        target,
        STEERING_NORMAL_LEFT,
        STEERING_NORMAL_RIGHT
    );
}

} // namespace

// ==========================================================
// INITIALIZATION
// ==========================================================

void navigation_init()
{
    previous_steering =
        STEERING_CENTER;

    std::cout
        << "\n"
        << "====================================\n"
        << "VELOZ SIMPLE LIDAR NAVIGATION\n"
        << "\n"
        << "40 LEFT | 90 CENTER | 140 RIGHT\n"
        << "LEFT wall  -> RIGHT steering\n"
        << "RIGHT wall -> LEFT steering\n"
        << "Servo step: "
        << MAX_STEERING_STEP
        << "\n"
        << "===================================="
        << std::endl;
}

// ==========================================================
// NAVIGATION
// ==========================================================

NavigationCommand navigation_update(
    float yaw_deg
)
{
    (void)yaw_deg;

    NavigationCommand command {};

    command.speed_percent =
        SPEED_NORMAL;

    command.steering_angle =
        previous_steering;

    command.emergency_stop =
        false;

    const LidarDistances d =
        lidar_get_distances();

    const bool front_valid =
        valid_distance(d.front_mm);

    const bool left_valid =
        valid_distance(d.left_mm);

    const bool right_valid =
        valid_distance(d.right_mm);

    // ======================================================
    // NO LIDAR DATA
    //
    // Continue with last steering.
    // Pico watchdog remains responsible for communication
    // safety.
    // ======================================================

    if (
        !front_valid &&
        !left_valid &&
        !right_valid
    )
    {
        command.speed_percent =
            SPEED_NORMAL;

        command.steering_angle =
            previous_steering;

        return command;
    }

    int target_steering =
        STEERING_CENTER;

    // ======================================================
    // FRONT WALL
    //
    // Choose whichever side has more open space.
    // ======================================================

    if (
        front_valid &&
        d.front_mm < FRONT_TURN_MM
    )
    {
        command.speed_percent =
            SPEED_TURN;

        // RIGHT side more open
        // -> turn RIGHT
        if (
            right_valid &&
            (
                !left_valid ||
                d.right_mm > d.left_mm
            )
        )
        {
            target_steering =
                STEERING_TURN_RIGHT;
        }

        // LEFT side more open
        // -> turn LEFT
        else if (left_valid)
        {
            target_steering =
                STEERING_TURN_LEFT;
        }

        else
        {
            target_steering =
                previous_steering;
        }
    }

    // ======================================================
    // NORMAL WALL AVOIDANCE
    // ======================================================

    else
    {
        target_steering =
            calculate_side_steering(d);

        if (
            target_steering !=
            STEERING_CENTER
        )
        {
            command.speed_percent =
                SPEED_CORRECTION;
        }
        else
        {
            command.speed_percent =
                SPEED_NORMAL;
        }
    }

    // ======================================================
    // EXTRA CLOSE SIDE PROTECTION
    // ======================================================

    if (
        left_valid &&
        d.left_mm < SIDE_DANGER_MM
    )
    {
        // LEFT wall dangerously close.
        // Strong RIGHT correction.

        target_steering =
            std::max(
                target_steering,
                STEERING_NORMAL_RIGHT
            );

        command.speed_percent =
            SPEED_CORRECTION;
    }

    if (
        right_valid &&
        d.right_mm < SIDE_DANGER_MM
    )
    {
        // RIGHT wall dangerously close.
        // Strong LEFT correction.

        target_steering =
            std::min(
                target_steering,
                STEERING_NORMAL_LEFT
            );

        command.speed_percent =
            SPEED_CORRECTION;
    }

    // ======================================================
    // SMOOTH SERVO
    // ======================================================

    const int steering =
        smooth_steering(
            previous_steering,
            target_steering
        );

    previous_steering =
        steering;

    command.steering_angle =
        steering;

    // ======================================================
    // DEBUG
    // ======================================================

    static int debug_counter = 0;

    if (++debug_counter >= 5)
    {
        debug_counter = 0;

        std::cout
            << "NAV"
            << " | F=" << d.front_mm
            << " | L=" << d.left_mm
            << " | R=" << d.right_mm
            << " | Target=" << target_steering
            << " | Servo=" << steering
            << " | Speed="
            << command.speed_percent
            << std::endl;
    }

    return command;
}