#pragma once

struct CameraDetection
{
    bool red_detected;
    bool green_detected;

    int red_x;
    int red_y;
    int red_area;

    int green_x;
    int green_y;
    int green_area;

    int frame_width;
    int frame_height;
};

bool camera_init();

bool camera_update();

CameraDetection camera_get_detection();

bool camera_is_ready();

void camera_close();