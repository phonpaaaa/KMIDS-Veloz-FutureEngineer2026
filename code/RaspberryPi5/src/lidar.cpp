#include "lidar.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;

namespace
{

// ==========================================================
// S3 DRIVER STATE
// ==========================================================

ILidarDriver* driver = nullptr;
IChannel* channel = nullptr;

std::atomic<bool> ready{false};
std::atomic<bool> scan_thread_running{false};

bool motor_started = false;
bool scan_started = false;

std::thread scan_thread;

// ==========================================================
// THREAD-SAFE DISTANCES
// ==========================================================

std::mutex distance_mutex;

LidarDistances distances
{
    -1.0f,
    -1.0f,
    -1.0f,
    -1.0f
};

std::atomic<std::uint64_t> scan_counter{0};

// ==========================================================
// S3 CONNECTION
// ==========================================================

constexpr const char* DEVICE =
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
    "407f98d9171ef11186e5c8e40f0f12f8-if00-port0";

constexpr sl_u32 BAUDRATE = 1000000;

// ==========================================================
// FILTER SETTINGS
// ==========================================================

constexpr float MIN_DISTANCE_MM = 80.0f;
constexpr float MAX_DISTANCE_MM = 6000.0f;

constexpr float SECTOR_HALF_WIDTH = 10.0f;

// ==========================================================
// PHYSICAL S3 ORIENTATION
// ==========================================================
//
// Verified from physical test:
//
// Robot FRONT = LiDAR 270°
// Robot RIGHT = LiDAR 180°
// Robot BACK  = LiDAR 90°
// Robot LEFT  = LiDAR 0°
// ==========================================================

constexpr float FRONT_ANGLE = 270.0f;
constexpr float RIGHT_ANGLE = 0.0f;
constexpr float BACK_ANGLE  = 90.0f;
constexpr float LEFT_ANGLE  = 180.0f;

// ==========================================================
// HELPERS
// ==========================================================

bool angle_in_sector(
    float angle,
    float center,
    float half_width
)
{
    float difference =
        std::fabs(angle - center);

    if (difference > 180.0f)
    {
        difference =
            360.0f - difference;
    }

    return difference <= half_width;
}

float median_distance(
    std::vector<float>& values
)
{
    if (values.empty())
    {
        return -1.0f;
    }

    std::sort(
        values.begin(),
        values.end()
    );

    const std::size_t middle =
        values.size() / 2;

    if ((values.size() % 2) == 0)
    {
        return
            (
                values[middle - 1] +
                values[middle]
            )
            / 2.0f;
    }

    return values[middle];
}

// ==========================================================
// PROCESS ONE SCAN
// ==========================================================

bool process_scan(
    sl_lidar_response_measurement_node_hq_t* nodes,
    std::size_t count
)
{
    std::vector<float> front_values;
    std::vector<float> left_values;
    std::vector<float> right_values;
    std::vector<float> back_values;

    front_values.reserve(512);
    left_values.reserve(512);
    right_values.reserve(512);
    back_values.reserve(512);

    for (std::size_t i = 0; i < count; ++i)
    {
        if (nodes[i].quality == 0)
        {
            continue;
        }

        const float distance_mm =
            static_cast<float>(
                nodes[i].dist_mm_q2
            )
            / 4.0f;

        if (
            distance_mm < MIN_DISTANCE_MM ||
            distance_mm > MAX_DISTANCE_MM
        )
        {
            continue;
        }

        float angle_deg =
            static_cast<float>(
                nodes[i].angle_z_q14
            )
            *
            90.0f
            /
            16384.0f;

        while (angle_deg >= 360.0f)
        {
            angle_deg -= 360.0f;
        }

        while (angle_deg < 0.0f)
        {
            angle_deg += 360.0f;
        }

        // FRONT
        if (
            angle_in_sector(
                angle_deg,
                FRONT_ANGLE,
                SECTOR_HALF_WIDTH
            )
        )
        {
            front_values.push_back(
                distance_mm
            );
        }

        // RIGHT
        if (
            angle_in_sector(
                angle_deg,
                RIGHT_ANGLE,
                SECTOR_HALF_WIDTH
            )
        )
        {
            right_values.push_back(
                distance_mm
            );
        }

        // BACK
        if (
            angle_in_sector(
                angle_deg,
                BACK_ANGLE,
                SECTOR_HALF_WIDTH
            )
        )
        {
            back_values.push_back(
                distance_mm
            );
        }

        // LEFT
        if (
            angle_in_sector(
                angle_deg,
                LEFT_ANGLE,
                SECTOR_HALF_WIDTH
            )
        )
        {
            left_values.push_back(
                distance_mm
            );
        }
    }

    const float front =
        median_distance(
            front_values
        );

    const float left =
        median_distance(
            left_values
        );

    const float right =
        median_distance(
            right_values
        );

    const float back =
        median_distance(
            back_values
        );

    {
        std::lock_guard<std::mutex> lock(
            distance_mutex
        );

        // IMPORTANT:
        // Every value belongs to THIS scan.
        // Missing sector becomes -1.
        distances.front_mm = front;
        distances.left_mm  = left;
        distances.right_mm = right;
        distances.back_mm  = back;
    }

    const bool updated =
        front > 0.0f ||
        left  > 0.0f ||
        right > 0.0f ||
        back  > 0.0f;

    if (updated)
    {
        scan_counter++;
    }

    return updated;
}

// ==========================================================
// BACKGROUND S3 THREAD
// ==========================================================

void scan_worker()
{
    std::cout
        << "S3 continuous scan thread started"
        << std::endl;

    int consecutive_failures = 0;

    while (scan_thread_running.load())
    {
        if (driver == nullptr)
        {
            break;
        }

        sl_lidar_response_measurement_node_hq_t
            nodes[8192]{};

        std::size_t count =
            sizeof(nodes) /
            sizeof(nodes[0]);

        const sl_result result =
            driver->grabScanDataHq(
                nodes,
                count,
                1000
            );

        // ==================================================
        // FAILED / TIMEOUT
        // ==================================================

        if (
            SL_IS_FAIL(result) ||
            count == 0
        )
        {
            ++consecutive_failures;

            if (
                consecutive_failures == 1 ||
                consecutive_failures % 10 == 0
            )
            {
                std::cerr
                    << "S3 READ MISS"
                    << " | result=0x"
                    << std::hex
                    << result
                    << std::dec
                    << " | failures="
                    << consecutive_failures
                    << std::endl;
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(10)
            );

            continue;
        }

        // ==================================================
        // GOOD SCAN
        // ==================================================

        consecutive_failures = 0;

        const sl_result sort_result =
            driver->ascendScanData(
                nodes,
                count
            );

        if (SL_IS_FAIL(sort_result))
        {
            std::cerr
                << "S3 SORT FAILED"
                << " | result=0x"
                << std::hex
                << sort_result
                << std::dec
                << std::endl;

            continue;
        }

        if (process_scan(nodes, count))
        {
            const std::uint64_t scan_number =
                scan_counter.load();

            if (
                scan_number > 0 &&
                (scan_number % 10) == 0
            )
            {
                LidarDistances d;

                {
                    std::lock_guard<std::mutex> lock(
                        distance_mutex
                    );

                    d = distances;
                }

                std::cout
                    << std::fixed
                    << std::setprecision(0)
                    << "S3 LIVE"
                    << " | Scan="
                    << scan_number
                    << " | F="
                    << d.front_mm
                    << " | L="
                    << d.left_mm
                    << " | R="
                    << d.right_mm
                    << " | B="
                    << d.back_mm
                    << std::endl;
            }
        }
    }

    std::cout
        << "S3 continuous scan thread stopped"
        << std::endl;
}

// ==========================================================
// INTERNAL CLEANUP
// ==========================================================

void cleanup_connection()
{
    ready = false;
    scan_thread_running = false;

    if (scan_thread.joinable())
    {
        scan_thread.join();
    }

    if (driver != nullptr)
    {
        if (scan_started)
        {
            driver->stop();
            scan_started = false;

            std::this_thread::sleep_for(
                std::chrono::milliseconds(20)
            );
        }

        if (motor_started)
        {
            driver->setMotorSpeed(0);
            motor_started = false;
        }

        driver->disconnect();

        delete driver;
        driver = nullptr;
    }

    if (channel != nullptr)
    {
        delete channel;
        channel = nullptr;
    }

    {
        std::lock_guard<std::mutex> lock(
            distance_mutex
        );

        distances =
        {
            -1.0f,
            -1.0f,
            -1.0f,
            -1.0f
        };
    }
}

} // namespace

// ==========================================================
// INITIALIZE
// ==========================================================

bool lidar_init()
{
    if (ready.load())
    {
        return true;
    }

    ready = false;
    scan_thread_running = false;

    motor_started = false;
    scan_started = false;

    scan_counter = 0;

    {
        std::lock_guard<std::mutex> lock(
            distance_mutex
        );

        distances =
        {
            -1.0f,
            -1.0f,
            -1.0f,
            -1.0f
        };
    }

    std::cout
        << "====================================\n"
        << "VELOZ - RPLIDAR S3\n"
        << "BACKGROUND SCAN MODE\n"
        << "====================================\n"
        << "Device:\n"
        << DEVICE
        << "\nBaud: "
        << BAUDRATE
        << "\n===================================="
        << std::endl;

    // ======================================================
    // 1. DRIVER
    // ======================================================

    std::cout
        << "[1] Creating LiDAR driver..."
        << std::endl;

    driver =
        *createLidarDriver();

    if (driver == nullptr)
    {
        std::cerr
            << "[1] DRIVER FAILED"
            << std::endl;

        return false;
    }

    std::cout
        << "[1] OK"
        << std::endl;

    // ======================================================
    // 2. SERIAL CHANNEL
    // ======================================================

    std::cout
        << "[2] Creating serial channel..."
        << std::endl;

    channel =
        *createSerialPortChannel(
            DEVICE,
            BAUDRATE
        );

    if (channel == nullptr)
    {
        std::cerr
            << "[2] CHANNEL FAILED"
            << std::endl;

        cleanup_connection();

        return false;
    }

    std::cout
        << "[2] OK"
        << std::endl;

    // ======================================================
    // 3. CONNECT
    // ======================================================

    std::cout
        << "[3] Connecting to S3..."
        << std::endl;

    sl_result result =
        driver->connect(
            channel
        );

    if (SL_IS_FAIL(result))
    {
        std::cerr
            << "[3] CONNECT FAILED"
            << " | result=0x"
            << std::hex
            << result
            << std::dec
            << std::endl;

        cleanup_connection();

        return false;
    }

    std::cout
        << "[3] CONNECTED"
        << std::endl;

    // ======================================================
    // 4. DEVICE INFO
    // ======================================================

    std::cout
        << "[4] Reading device info..."
        << std::endl;

    sl_lidar_response_device_info_t info{};

    result =
        driver->getDeviceInfo(
            info,
            2000
        );

    if (SL_IS_FAIL(result))
    {
        std::cerr
            << "[4] DEVICE INFO TIMEOUT"
            << " | result=0x"
            << std::hex
            << result
            << std::dec
            << std::endl;

        std::cerr
            << "[4] Continuing without device info."
            << std::endl;
    }
    else
    {
        std::cout
            << "[4] DEVICE INFO OK"
            << std::endl;

        std::cout
            << "Firmware: "
            << (info.firmware_version >> 8)
            << "."
            << (info.firmware_version & 0xFF)
            << std::endl;

        std::cout
            << "Hardware revision: "
            << static_cast<int>(
                info.hardware_version
            )
            << std::endl;
    }

    // ======================================================
    // 5. HEALTH
    // ======================================================

    std::cout
        << "[5] Checking health..."
        << std::endl;

    sl_lidar_response_device_health_t health{};

    result =
        driver->getHealth(
            health,
            2000
        );

    if (SL_IS_FAIL(result))
    {
        std::cerr
            << "[5] HEALTH TIMEOUT"
            << " | result=0x"
            << std::hex
            << result
            << std::dec
            << std::endl;

        std::cerr
            << "[5] Continuing to scan test."
            << std::endl;
    }
    else
    {
        std::cout
            << "[5] Status: "
            << static_cast<int>(
                health.status
            )
            << " | Error code: "
            << health.error_code
            << std::endl;

        if (
            health.status ==
            SL_LIDAR_STATUS_ERROR
        )
        {
            std::cerr
                << "[5] S3 REPORTED HEALTH ERROR"
                << std::endl;

            cleanup_connection();

            return false;
        }
    }

    // ======================================================
    // 6. MOTOR
    // ======================================================

    std::cout
        << "[6] Starting motor..."
        << std::endl;

    result =
        driver->setMotorSpeed();

    if (SL_IS_FAIL(result))
    {
        std::cerr
            << "[6] MOTOR FAILED"
            << " | result=0x"
            << std::hex
            << result
            << std::dec
            << std::endl;

        cleanup_connection();

        return false;
    }

    motor_started = true;

    std::cout
        << "[6] MOTOR STARTED"
        << std::endl;

    std::this_thread::sleep_for(
        std::chrono::milliseconds(500)
    );

    // ======================================================
    // 7. START SCAN
    // ======================================================

    std::cout
        << "[7] Starting scan..."
        << std::endl;

    result =
        driver->startScan(
            0,
            1
        );

    if (SL_IS_FAIL(result))
    {
        std::cerr
            << "[7] SCAN FAILED"
            << " | result=0x"
            << std::hex
            << result
            << std::dec
            << std::endl;

        cleanup_connection();

        return false;
    }

    scan_started = true;

    std::cout
        << "[7] SCAN STARTED"
        << std::endl;

    // ======================================================
    // 8. WARM UP
    // ======================================================

    std::cout
        << "[8] Warming up..."
        << std::endl;

    std::this_thread::sleep_for(
        std::chrono::milliseconds(1000)
    );

    // ======================================================
    // 9. START THREAD
    // ======================================================

    scan_thread_running = true;

    scan_thread =
        std::thread(
            scan_worker
        );

    ready = true;

    std::cout
        << "[9] READY"
        << std::endl;

    std::cout
        << "===================================="
        << std::endl;

    return true;
}

// ==========================================================
// UPDATE
// ==========================================================

bool lidar_update()
{
    if (!ready.load())
    {
        return false;
    }

    static std::uint64_t previous_scan_counter = 0;

    const std::uint64_t current =
        scan_counter.load();

    if (
        current !=
        previous_scan_counter
    )
    {
        previous_scan_counter =
            current;

        return true;
    }

    return false;
}

// ==========================================================
// GET DISTANCES
// ==========================================================

LidarDistances lidar_get_distances()
{
    std::lock_guard<std::mutex> lock(
        distance_mutex
    );

    return distances;
}

// ==========================================================
// READY
// ==========================================================

bool lidar_is_ready()
{
    return ready.load();
}

// ==========================================================
// CLOSE
// ==========================================================

void lidar_close()
{
    std::cout
        << "Closing RPLIDAR S3..."
        << std::endl;

    cleanup_connection();

    std::cout
        << "RPLIDAR S3 closed"
        << std::endl;
}