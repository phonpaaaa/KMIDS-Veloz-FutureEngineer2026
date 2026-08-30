#include "pico_i2c.h"

#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

namespace
{

constexpr int PICO_ADDRESS = 0x39;

constexpr const char* I2C_DEVICE =
    "/dev/i2c-1";

int i2c_file = -1;

}

// ==========================================================
// INIT
// ==========================================================

bool pico_i2c_init()
{
    i2c_file =
        open(
            I2C_DEVICE,
            O_RDWR
        );

    if (i2c_file < 0)
    {
        std::cerr
            << "Failed to open "
            << I2C_DEVICE
            << std::endl;

        return false;
    }

    // Reduce retry count so a bad transaction
    // does not stall the robot for too long.
    ioctl(
        i2c_file,
        I2C_RETRIES,
        1
    );

    // Kernel I2C timeout.
    // Unit is approximately 10 ms on Linux adapters
    // that support this option.
    ioctl(
        i2c_file,
        I2C_TIMEOUT,
        2
    );

    std::cout
        << "Connected to Pico at 0x39"
        << std::endl;

    return true;
}

// ==========================================================
// SEND COMMAND
// ==========================================================

bool pico_i2c_send_command(
    const PicoCommand& command
)
{
    if (i2c_file < 0)
    {
        return false;
    }

    unsigned char data[3];

    data[0] =
        static_cast<unsigned char>(
            command.speed_percent
        );

    data[1] =
        command.steering_angle;

    data[2] =
        command.emergency_stop;

    struct i2c_msg message {};

    message.addr =
        PICO_ADDRESS;

    message.flags =
        0;

    message.len =
        sizeof(data);

    message.buf =
        data;

    struct i2c_rdwr_ioctl_data packet {};

    packet.msgs =
        &message;

    packet.nmsgs =
        1;

    const int result =
        ioctl(
            i2c_file,
            I2C_RDWR,
            &packet
        );

    if (result != 1)
    {
        std::cerr
            << "PICO I2C WRITE FAILED"
            << std::endl;

        return false;
    }

    return true;
}

// ==========================================================
// READ TELEMETRY
// ==========================================================

bool pico_i2c_read_telemetry(
    PicoTelemetry& telemetry
)
{
    if (i2c_file < 0)
    {
        return false;
    }

    unsigned char buffer[
        sizeof(PicoTelemetry)
    ] = {};

    struct i2c_msg message {};

    message.addr =
        PICO_ADDRESS;

    message.flags =
        I2C_M_RD;

    message.len =
        sizeof(buffer);

    message.buf =
        buffer;

    struct i2c_rdwr_ioctl_data packet {};

    packet.msgs =
        &message;

    packet.nmsgs =
        1;

    const int result =
        ioctl(
            i2c_file,
            I2C_RDWR,
            &packet
        );

    if (result != 1)
    {
        return false;
    }

    std::memcpy(
        &telemetry,
        buffer,
        sizeof(PicoTelemetry)
    );

    return true;
}