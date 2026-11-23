#pragma once

// ---------------------------------------------------------------------------
// mailbox  —  single-slot latest-value store for gamepad_wire_t.
//
// The UART RX task writes the most recent controller snapshot here.
// The gait task reads it on every 100 Hz tick.
//
// Semantics: writer always overwrites (never blocks on full), reader always
// gets the latest value.  No condition variable — the gait task is
// timer-driven and snapshots whatever is current when it wakes.
// ---------------------------------------------------------------------------

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "protocol.h"
#include <stdbool.h>

typedef struct {
  gamepad_wire_t state;
  bool valid; // false until the first good frame arrives
  SemaphoreHandle_t mutex;
} gamepad_mailbox_t;

// Call once from app_main before starting tasks.
void mailbox_init(gamepad_mailbox_t *mb);

// Write the latest snapshot.  Called by the UART RX task.
void mailbox_write(gamepad_mailbox_t *mb, const gamepad_wire_t *src);

// Read the latest snapshot into *dst.
// Returns true if at least one frame has been received.
bool mailbox_read(gamepad_mailbox_t *mb, gamepad_wire_t *dst);
