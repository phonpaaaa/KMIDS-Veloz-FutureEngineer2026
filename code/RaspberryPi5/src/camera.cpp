#include "camera.h"

#include <algorithm>
#include <iostream>
#include <mutex>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

namespace
{

bool ready = false;

cv::VideoCapture camera;

std::mutex detection_mutex;

CameraDetection detection
{
    false,
    false,

    -1,
    -1,
    0,

    -1,
    -1,
    0,

    0,
    0
};

// ==========================================================
// SETTINGS
// ==========================================================

constexpr int CAMERA_INDEX = 0;

constexpr int FRAME_WIDTH  = 640;
constexpr int FRAME_HEIGHT = 480;

// Ignore tiny blobs/noise.
constexpr double MIN_BLOB_AREA = 500.0;

// ==========================================================
// FIND LARGEST COLORED OBJECT
// ==========================================================

bool find_largest_blob(
    const cv::Mat& mask,
    int& out_x,
    int& out_y,
    int& out_area
)
{
    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(
        mask,
        contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE
    );

    double largest_area = 0.0;
    cv::Rect largest_rect;

    for (const auto& contour : contours)
    {
        const double area =
            cv::contourArea(contour);

        if (area < MIN_BLOB_AREA)
        {
            continue;
        }

        if (area > largest_area)
        {
            largest_area = area;
            largest_rect =
                cv::boundingRect(contour);
        }
    }

    if (largest_area <= 0.0)
    {
        out_x = -1;
        out_y = -1;
        out_area = 0;

        return false;
    }

    out_x =
        largest_rect.x +
        largest_rect.width / 2;

    out_y =
        largest_rect.y +
        largest_rect.height / 2;

    out_area =
        static_cast<int>(
            largest_area
        );

    return true;
}

} // namespace

// ==========================================================
// INITIALIZE
// ==========================================================

bool camera_init()
{
    if (ready)
    {
        return true;
    }

    std::cout
        << "====================================\n"
        << "VELOZ - CAMERA\n"
        << "Raspberry Pi 5\n"
        << "===================================="
        << std::endl;

    camera.open(
        CAMERA_INDEX,
        cv::CAP_V4L2
    );

    if (!camera.isOpened())
    {
        std::cerr
            << "ERROR: Could not open camera."
            << std::endl;

        ready = false;

        return false;
    }

    camera.set(
        cv::CAP_PROP_FRAME_WIDTH,
        FRAME_WIDTH
    );

    camera.set(
        cv::CAP_PROP_FRAME_HEIGHT,
        FRAME_HEIGHT
    );

    camera.set(
        cv::CAP_PROP_BUFFERSIZE,
        1
    );

    {
        std::lock_guard<std::mutex> lock(
            detection_mutex
        );

        detection.frame_width =
            FRAME_WIDTH;

        detection.frame_height =
            FRAME_HEIGHT;
    }

    ready = true;

    std::cout
        << "Camera ready."
        << std::endl;

    return true;
}

// ==========================================================
// UPDATE
// ==========================================================

bool camera_update()
{
    if (!ready)
    {
        return false;
    }

    cv::Mat frame;

    if (!camera.read(frame))
    {
        std::cerr
            << "Camera frame read failed."
            << std::endl;

        return false;
    }

    if (frame.empty())
    {
        return false;
    }

    cv::Mat hsv;

    cv::cvtColor(
        frame,
        hsv,
        cv::COLOR_BGR2HSV
    );

    // ======================================================
    // RED MASK
    //
    // Red wraps around HSV hue, so use two ranges.
    // ======================================================

    cv::Mat red_mask_1;
    cv::Mat red_mask_2;
    cv::Mat red_mask;

    cv::inRange(
        hsv,
        cv::Scalar(0, 100, 70),
        cv::Scalar(10, 255, 255),
        red_mask_1
    );

    cv::inRange(
        hsv,
        cv::Scalar(170, 100, 70),
        cv::Scalar(179, 255, 255),
        red_mask_2
    );

    red_mask =
        red_mask_1 |
        red_mask_2;

    // ======================================================
    // GREEN MASK
    // ======================================================

    cv::Mat green_mask;

    cv::inRange(
        hsv,
        cv::Scalar(35, 70, 50),
        cv::Scalar(90, 255, 255),
        green_mask
    );

    // ======================================================
    // REMOVE SMALL NOISE
    // ======================================================

    const cv::Mat kernel =
        cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(5, 5)
        );

    cv::morphologyEx(
        red_mask,
        red_mask,
        cv::MORPH_OPEN,
        kernel
    );

    cv::morphologyEx(
        green_mask,
        green_mask,
        cv::MORPH_OPEN,
        kernel
    );

    // ======================================================
    // DETECT LARGEST RED + GREEN OBJECTS
    // ======================================================

    CameraDetection new_detection{};

    new_detection.frame_width =
        frame.cols;

    new_detection.frame_height =
        frame.rows;

    new_detection.red_detected =
        find_largest_blob(
            red_mask,
            new_detection.red_x,
            new_detection.red_y,
            new_detection.red_area
        );

    new_detection.green_detected =
        find_largest_blob(
            green_mask,
            new_detection.green_x,
            new_detection.green_y,
            new_detection.green_area
        );

    {
        std::lock_guard<std::mutex> lock(
            detection_mutex
        );

        detection =
            new_detection;
    }

    return true;
}

// ==========================================================
// GET DETECTION
// ==========================================================

CameraDetection camera_get_detection()
{
    std::lock_guard<std::mutex> lock(
        detection_mutex
    );

    return detection;
}

// ==========================================================
// READY
// ==========================================================

bool camera_is_ready()
{
    return ready;
}

// ==========================================================
// CLOSE
// ==========================================================

void camera_close()
{
    ready = false;

    if (camera.isOpened())
    {
        camera.release();
    }

    {
        std::lock_guard<std::mutex> lock(
            detection_mutex
        );

        detection =
        {
            false,
            false,

            -1,
            -1,
            0,

            -1,
            -1,
            0,

            0,
            0
        };
    }

    std::cout
        << "Camera closed."
        << std::endl;
}