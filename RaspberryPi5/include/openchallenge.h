#pragma once

// ==========================================================
// VELOZ - WRO FUTURE ENGINEER OPEN CHALLENGE
// ==========================================================

// Start Open Challenge.
//
// This function owns the control loop and blocks until:
// - 12 corners are completed
// - open_challenge_stop() is called
// - a safety condition stops the robot
void open_challenge_start();

// Request stop.
void open_challenge_stop();

// True while challenge loop is active.
bool open_challenge_is_running();