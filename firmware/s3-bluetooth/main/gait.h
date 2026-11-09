#pragma once

#include "robot.h"
#include <stdbool.h>

// ---------------------------------------------------------------------------
// vec3_t — a foot position in body frame.
//
//   x = forward  (+x is ahead of robot)
//   y = left     (+y is robot's left side)
//   z = up       (z = 0 is ground level, z = -BODY_HEIGHT is nominal foot z)
//
// This is the only type gait.c produces.  IK will consume it later.
// ---------------------------------------------------------------------------
typedef struct {
  float x, y, z;
} vec3_t;

// ---------------------------------------------------------------------------
// gait_leg_t — per-leg oscillator state.
//
// The gait module tracks three body-frame positions per leg:
//   p_contact — where the foot is right now (updated continuously in stance,
//               snapped to p_target on touchdown)
//   p_lift    — snapshot of p_contact at the moment the foot left the ground
//   p_target  — where the foot should land (computed by Raibert at liftoff)
//
// in_swing is cached so we can detect liftoff and touchdown transitions by
// comparing the previous value against the newly computed phase state.
// ---------------------------------------------------------------------------
typedef struct {
  vec3_t p_contact;
  vec3_t p_lift;
  vec3_t p_target;
  bool in_swing;
} gait_leg_t;

// ---------------------------------------------------------------------------
// gait_t — full oscillator state.
//
// master_phase ∈ [0, 1) is a single shared clock.  Each leg derives its own
// effective phase by adding a fixed offset (defined in gait.c).  This keeps
// all legs on one clock so gait switching later is a simple offset change.
//
// beta is the duty factor: fraction of one cycle spent in stance.
//   beta = 0.5 → trot (each leg is in stance exactly half the time)
//
// freq drives how fast master_phase advances.  At fixed freq the robot takes
// bigger steps as commanded speed increases (Raibert feedforward grows).
// ---------------------------------------------------------------------------
typedef struct {
  gait_leg_t legs[NUM_LEGS];
  float master_phase;
  float beta;
  float freq; // Hz
} gait_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// Initialise gait state with trot defaults.  Feet start at their nominal
// rest positions (directly below each hip socket at BODY_HEIGHT depth).
void gait_init(gait_t *g);

// Advance the gait by dt seconds given body-frame velocity commands:
//   vx    — forward speed  [m/s]  (positive = forward)
//   vy    — lateral speed  [m/s]  (positive = left / strafe)
//   omega — yaw rate       [rad/s] (positive = turn left, right-hand z-up)
//
// Writes the foot target for each leg into foot_out[NUM_LEGS].
// Pure math — no FreeRTOS, no hardware, no globals.
void gait_update(gait_t *g, float vx, float vy, float omega, float dt,
                 vec3_t foot_out[NUM_LEGS]);
