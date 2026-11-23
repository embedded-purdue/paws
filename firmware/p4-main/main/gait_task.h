#pragma once

// ---------------------------------------------------------------------------
// gait_task  —  100 Hz control loop for the ESP32-P4.
//
// Runs as a FreeRTOS task pinned to CPU1.  On each tick:
//   1. Snapshot the gamepad mailbox.
//   2. Apply deadzone and scale axes to body-frame velocity commands.
//   3. Call gait_update() to advance the oscillator.
//   4. (IK placeholder — joint angles go here once the IK module exists.)
// ---------------------------------------------------------------------------

#include "mailbox.h"

// gait_task_start — launch the 100 Hz gait task on CPU1.
void gait_task_start(gamepad_mailbox_t *mailbox);
