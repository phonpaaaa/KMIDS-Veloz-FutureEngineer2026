#include "pi_i2c.h"
#include "config.h"

#include <cstring>
#include <cstdio>

#include "hardware/i2c.h"
#include "pico/i2c_slave.h"
#include "pico/stdlib.h"

namespace
{

// ==========================================================
// I2C PORT
//
// Pico I2C1
// GP2 = SDA
// GP3 = SCL
// ==========================================================

constexpr i2c_inst_t* PI_I2C_PORT = i2c1;

// ==========================================================
// COMMAND FROM RASPBERRY PI
// ==========================================================

PiCommand latest_command = {
    0,
    SERVO_CENTER,
    1
};

volatile bool command_received = false;

unsigned char receive_buffer[sizeof(PiCommand)] = {};

volatile unsigned int receive_index = 0;

// ==========================================================
// TELEMETRY TO RASPBERRY PI
// ==========================================================

PicoTelemetry latest_telemetry = {};

unsigned char telemetry_buffer[sizeof(PicoTelemetry)] = {};

volatile unsigned int telemetry_index = 0;

// ==========================================================
// I2C SLAVE CALLBACK
// ==========================================================

void i2c_slave_handler(
    i2c_inst_t* i2c,
    i2c_slave_event_t event
)
{
    switch (event)
    {
        // ==================================================
        // RECEIVE DATA FROM PI
        // ==================================================

        case I2C_SLAVE_RECEIVE:
        {
            const unsigned char value =
                i2c_read_byte_raw(i2c);

            if (
                receive_index <
                sizeof(PiCommand)
            )
            {
                receive_buffer[receive_index] =
                    value;

                receive_index++;
            }

            break;
        }

        // ==================================================
        // PI REQUESTS TELEMETRY
        // ==================================================

        case I2C_SLAVE_REQUEST:
        {
            if (
                telemetry_index >=
                sizeof(PicoTelemetry)
            )
            {
                telemetry_index = 0;
            }

            i2c_write_byte_raw(
                i2c,
                telemetry_buffer[telemetry_index]
            );

            telemetry_index++;

            break;
        }

        // ==================================================
        // TRANSACTION FINISHED
        // ==================================================

        case I2C_SLAVE_FINISH:
        {
            // ----------------------------------------------
            // Pi sent one complete command packet
            // ----------------------------------------------

            if (
                receive_index ==
                sizeof(PiCommand)
            )
            {
                std::memcpy(
                    &latest_command,
                    receive_buffer,
                    sizeof(PiCommand)
                );

                command_received = true;
            }

            // Reset packet state
            receive_index = 0;
            telemetry_index = 0;

            break;
        }

        default:
        {
            break;
        }
    }
}

}

// ==========================================================
// INITIALIZATION
// ==========================================================

void pi_i2c_init()
{
    // ======================================================
    // INITIALIZE I2C1
    // ======================================================

    i2c_init(
        PI_I2C_PORT,
        PI_I2C_BAUDRATE
    );

    // ======================================================
    // GPIO CONFIGURATION
    // ======================================================

    gpio_set_function(
        PI_SDA_PIN,
        GPIO_FUNC_I2C
    );

    gpio_set_function(
        PI_SCL_PIN,
        GPIO_FUNC_I2C
    );

    // Internal pull-ups.
    //
    // External 4.7k pull-ups are still preferable
    // for a robust Pi <-> Pico I2C connection.
    gpio_pull_up(
        PI_SDA_PIN
    );

    gpio_pull_up(
        PI_SCL_PIN
    );

    // ======================================================
    // INITIAL TELEMETRY BUFFER
    // ======================================================

    std::memcpy(
        telemetry_buffer,
        &latest_telemetry,
        sizeof(PicoTelemetry)
    );

    // ======================================================
    // START PICO AS I2C SLAVE
    // ======================================================

    i2c_slave_init(
        PI_I2C_PORT,
        PICO_I2C_ADDRESS,
        &i2c_slave_handler
    );

    printf(
        "Pi I2C slave initialized\n"
    );

    printf(
        "Address: 0x%02X\n",
        PICO_I2C_ADDRESS
    );

    printf(
        "SDA: GP%u | SCL: GP%u | Baud: %u\n",
        PI_SDA_PIN,
        PI_SCL_PIN,
        PI_I2C_BAUDRATE
    );
}

// ==========================================================
// UPDATE
// ==========================================================

void pi_i2c_update()
{
    // I2C communication is interrupt-driven.
}

// ==========================================================
// COMMAND ACCESS
// ==========================================================

PiCommand pi_i2c_get_command()
{
    return latest_command;
}

bool pi_i2c_command_received()
{
    return command_received;
}

void pi_i2c_clear_command_flag()
{
    command_received = false;
}

// ==========================================================
// TELEMETRY UPDATE
// ==========================================================

void pi_i2c_set_telemetry(
    const PicoTelemetry& telemetry
)
{
    latest_telemetry =
        telemetry;

    std::memcpy(
        telemetry_buffer,
        &latest_telemetry,
        sizeof(PicoTelemetry)
    );
}