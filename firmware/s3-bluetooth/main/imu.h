#pragma once

#include <stdbool.h>

// ---------------------------------------------------------------------------
// IMU public interface (BNO085 rotation vector, Euler angles in radians)
// ---------------------------------------------------------------------------

typedef struct {
  float roll;
  float pitch;
  float yaw;
  bool valid; // false until the first reading arrives
} imu_state_t;

// Spawn the BNO085 FreeRTOS task pinned to core 1. Call once from app_main.
void imu_start(void);

// Thread-safe snapshot of the latest IMU reading.
// Returns false (and leaves *out unchanged) if no reading has arrived yet.
bool imu_get(imu_state_t *out);
