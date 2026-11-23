#pragma once

// ---------------------------------------------------------------------------
// Physical constants extracted from the MJCF model (d1.xml).
// All units: metres, radians.
// Body frame convention: x = forward, y = left, z = up.
// ---------------------------------------------------------------------------

#define NUM_LEGS 4
#define LEG_FL 0 // front-left
#define LEG_FR 1 // front-right
#define LEG_RL 2 // rear-left
#define LEG_RR 3 // rear-right

// ---------------------------------------------------------------------------
// Hip (thigh pivot) positions in body frame.
//
// These are the HIP joint locations after accounting for the ABAD lateral
// offset (abad_pos + hip_offset_y).  They are the natural reference point
// for Raibert foot placement because the sagittal leg motion pivots here.
//
// From the MJCF:
//   ABAD at (±0.17449, ±0.062, 0)
//   HIP  offset from ABAD: (0, ±0.09741, 0)   → total y = ±0.1594
//
// Order: FL, FR, RL, RR
// ---------------------------------------------------------------------------
static const float HIP_X[NUM_LEGS] = {0.17449f, 0.17449f, -0.17449f, -0.17449f};
static const float HIP_Y[NUM_LEGS] = {0.15941f, -0.15940f, 0.15941f, -0.15940f};

// ---------------------------------------------------------------------------
// Link lengths (from MJCF child body offsets along z).
// ---------------------------------------------------------------------------
#define L_THIGH 0.20000f   // HIP  → KNEE  [m]
#define L_SHIN 0.21366f    // KNEE → FOOT  [m]
#define FOOT_RADIUS 0.022f // collision sphere radius [m]

// ---------------------------------------------------------------------------
// Nominal stance geometry.
//
// BODY_HEIGHT: CoM above ground when standing in the home pose.
//   Derived: foot_z_from_hip (−0.303 m at hip=0.8, knee=−1.5) + FOOT_RADIUS
//   = 0.303 + 0.022 ≈ 0.325 m
//
// STEP_HEIGHT: how high the foot lifts during swing.
//   Conservative starting value; tune upward if the robot scuffs.
// ---------------------------------------------------------------------------
#define BODY_HEIGHT 0.325f
#define STEP_HEIGHT 0.06f
