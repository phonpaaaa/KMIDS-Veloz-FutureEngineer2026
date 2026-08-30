#ifndef NAVIGATION_H
#define NAVIGATION_H

struct NavigationCommand
{
    int speed_percent;
    int steering_angle;
    bool emergency_stop;
};

// Reset navigation state.
void navigation_init();

// Compute the next motor + steering command.
// yaw_deg is reserved for later IMU-assisted navigation.
NavigationCommand navigation_update(
    float yaw_deg
);

#endif