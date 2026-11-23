#pragma once

// ---------------------------------------------------------------------------
// PAWS gamepad wire protocol  —  shared by S3 (IDF) and P4 (Linux)
//
// Frame layout on UART, 21 bytes total at 100 Hz:
//
//   byte 0    : PROTO_SYNC0  (0xA5)
//   byte 1    : PROTO_SYNC1  (0x5A)
//   bytes 2-19: gamepad_wire_t payload, 18 bytes, little-endian
//   byte 20   : CRC-8/SMBUS over bytes 2-19 only
//
// CRC-8/SMBUS: poly = 0x07, init = 0x00, no input/output reflection.
//
// At 921600 baud, 1 frame = ~0.24 ms on the wire.
// At 100 Hz, wire utilisation ≈ 2.4 %.
// ---------------------------------------------------------------------------

#include <stddef.h>
#include <stdint.h>

#define PROTO_SYNC0 0xA5u
#define PROTO_SYNC1 0x5Au
#define PROTO_PAYLOAD_LEN 18
#define PROTO_FRAME_LEN 21 // 2 sync + 18 payload + 1 CRC

// ---------------------------------------------------------------------------
// gamepad_wire_t  —  compact on-wire representation of a controller snapshot.
//
// Axes use the same sign/range as Bluepad32 / Xbox BLE:
//   axis_x/y/rx/ry: [-512, 511]
//   brake/throttle: [0, 1023]
//   buttons: Bluepad32 bitmask (BUTTON_A = 0x0001, etc.)
//   dpad:    Bluepad32 hat value
//   connected: 1 when a controller is paired, 0 otherwise
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
  int16_t axis_x;    // left stick X
  int16_t axis_y;    // left stick Y
  int16_t axis_rx;   // right stick X
  int16_t axis_ry;   // right stick Y
  uint16_t brake;    // left trigger
  uint16_t throttle; // right trigger
  uint32_t buttons;
  uint8_t dpad;
  uint8_t connected;
} gamepad_wire_t;

// Both translation units must see the same 18-byte layout.
_Static_assert(sizeof(gamepad_wire_t) == PROTO_PAYLOAD_LEN,
               "gamepad_wire_t must be exactly 18 bytes");

// ---------------------------------------------------------------------------
// proto_crc8 — CRC-8/SMBUS over `len` bytes of `data`.
// Inlined so it compiles into both IDF and Linux builds with no extra unit.
// ---------------------------------------------------------------------------
static inline uint8_t proto_crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00u;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
  }
  return crc;
}
