#pragma once

// ---------------------------------------------------------------------------
// uart_bridge  —  100 Hz fixed-rate UART TX relay from BT gamepad to P4.
//
// Runs as a FreeRTOS task pinned to CPU0 (alongside BT).  Every 10 ms it
// snapshots the shared gamepad state, packs a protocol.h wire frame, and
// writes it to UART1 (TX = GPIO 37, RX = GPIO 38, 921600 baud).
//
// The P4 is the sole consumer; the S3 never reads UART in this role.
// RX is configured anyway so the UART driver is symmetric and the wire is
// ready if we ever need a back-channel.
// ---------------------------------------------------------------------------

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "gamepad.h"

// uart_bridge_start — initialise UART1 and launch the TX task.
//
// Parameters:
//   mutex  — the same mutex that guards *state in main.c.
//   state  — pointer to the live gamepad_state_t (never freed for the
//             lifetime of the program).
//
// Call once from app_main before btstack_run_loop_execute().
void uart_bridge_start(SemaphoreHandle_t mutex, const gamepad_state_t *state);
