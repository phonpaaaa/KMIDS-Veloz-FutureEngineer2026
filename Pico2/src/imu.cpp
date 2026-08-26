#include "imu.h"

#include <stdio.h>

#include "bno08x.h"
#include "utils.h"
#include "pico/stdlib.h"

namespace
{
BNO08x imu_sensor;

i2c_inst_t* imu_i2c = nullptr;

float yaw_rad = 0.0f;
float pitch_rad = 0.0f;
float roll_rad = 0.0f;

bool imu_ready = false;

constexpr float RAD_TO_DEG =
    180.0f / 3.14159265358979323846f;
}

bool imu_init()
{
    printf("Initializing BNO085...\n");

    initI2C(imu_i2c, false);

    while (
        !imu_sensor.begin(
            CONFIG::BNO08X_ADDR,
            imu_i2c
        )
    )
    {
        printf(
            "BNO085 not detected at address 0x%02X\n",
            CONFIG::BNO08X_ADDR
        );

        scan_i2c_bus();
        sleep_ms(1000);
    }

    printf("BNO085 detected\n");

    if (!imu_sensor.enableRotationVector())
    {
        printf("Failed to enable rotation vector\n");
        imu_ready = false;
        return false;
    }

    imu_ready = true;

    printf("Rotation vector enabled\n");

    return true;
}

bool imu_update()
{
    if (!imu_ready)
    {
        return false;
    }

    if (!imu_sensor.getSensorEvent())
    {
        return false;
    }

    if (
        imu_sensor.getSensorEventID() !=
        SENSOR_REPORTID_ROTATION_VECTOR
    )
    {
        return false;
    }

    yaw_rad = imu_sensor.getYaw();
    pitch_rad = imu_sensor.getPitch();
    roll_rad = imu_sensor.getRoll();

    return true;
}

float imu_get_yaw_rad()
{
    return yaw_rad;
}

float imu_get_pitch_rad()
{
    return pitch_rad;
}

float imu_get_roll_rad()
{
    return roll_rad;
}

float imu_get_yaw_deg()
{
    return yaw_rad * RAD_TO_DEG;
}

float imu_get_pitch_deg()
{
    return pitch_rad * RAD_TO_DEG;
}

float imu_get_roll_deg()
{
    return roll_rad * RAD_TO_DEG;
}

bool imu_is_ready()
{
    return imu_ready;
}