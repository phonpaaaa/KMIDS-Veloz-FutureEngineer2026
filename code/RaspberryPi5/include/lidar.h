#ifndef LIDAR_H
#define LIDAR_H

// ==========================================================
// VELOZ - RPLIDAR S3
// Raspberry Pi 5
// ==========================================================

struct LidarDistances
{
    float front_mm;
    float left_mm;
    float right_mm;
    float back_mm;
};

// ==========================================================
// INITIALIZATION
// ==========================================================

// Connect to RPLIDAR S3
// Serial speed: 1,000,000 baud
bool lidar_init();

// ==========================================================
// SCANNING
// ==========================================================

// Grab one complete scan from the S3,
// process the scan,
// and update front/left/right/back.
//
// Returns:
// true  = new valid scan received
// false = scan failed / timed out
bool lidar_update();

// ==========================================================
// DATA
// ==========================================================

// Return latest processed distances.
//
// Invalid/unavailable direction = -1.0f
LidarDistances lidar_get_distances();

// ==========================================================
// STATUS
// ==========================================================

// true if S3 initialized successfully
bool lidar_is_ready();

// ==========================================================
// SHUTDOWN
// ==========================================================

// Stop scan
// Stop motor
// Disconnect and release driver
void lidar_close();

#endif