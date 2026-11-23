#pragma once

// ---------------------------------------------------------------------------
// gamepad_state_t  —  in-memory snapshot of a connected Xbox controller.
//
// Written by the Bluepad32 BT callback on CPU0 (mutex-protected).
// Read by the gait task (CPU1) and the UART bridge task (CPU0).
//
// Types match Bluepad32's uni_gamepad_t fields verbatim to avoid casts at
// the BT callback site.
// ---------------------------------------------------------------------------

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  int32_t axis_x;   // left  stick X  [-512, 511]
  int32_t axis_y;   // left  stick Y  [-512, 511]
  int32_t axis_rx;  // right stick X  [-512, 511]
  int32_t axis_ry;  // right stick Y  [-512, 511]
  int32_t brake;    // L2 / left trigger  [0, 1023]
  int32_t throttle; // R2 / right trigger [0, 1023]
  uint16_t buttons; // BUTTON_A … BUTTON_THUMB_R bitmask
  uint8_t dpad;     // DPAD_UP … DPAD_LEFT bitmask
  bool connected;
} gamepad_state_t;
