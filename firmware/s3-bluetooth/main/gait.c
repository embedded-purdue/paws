#include "gait.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const float PHASE_OFFSET[NUM_LEGS] = {
    [LEG_FL] = 0.0f,
    [LEG_FR] = 0.5f,
    [LEG_RL] = 0.5f,
    [LEG_RR] = 0.0f,
};

// ---------------------------------------------------------------------------
// lerp — linear interpolation between a and b by t ∈ [0,1].
// ---------------------------------------------------------------------------
static inline float lerp(float a, float b, float t) { return a + (b - a) * t; }

// ---------------------------------------------------------------------------
// raibert_target — compute the foot landing position for leg `leg`.
//
// The Raibert heuristic answers: given that the foot is about to leave the
// ground, where should it land?
//
// Answer: under where the hip will be at MID-stance — not at touchdown or at
// liftoff, but exactly halfway through the stance phase.  This is the neutral
// point where the leg is vertical and bears load symmetrically.
//
// The hip's velocity in the body frame has two components:
//   1. The body's commanded linear velocity (vx, vy).
//   2. A tangential component from yaw: ω × r_hip.
//
//      Cross product (ω_z k̂) × (rx î + ry ĵ):
//        = ω*(ry î  × ĵ × ĵ ... ) — standard 3D cross:
//        x-component: -ω * ry   (where ry = HIP_Y[leg])
//        y-component: +ω * rx   (where rx = HIP_X[leg])
//
//      So: v_hip_x = vx - omega * HIP_Y[leg]
//          v_hip_y = vy + omega * HIP_X[leg]
//
// The landing target is then:
//   p_target = hip_pos + v_hip * (T_stance / 2)
//
// where T_stance = beta / freq.
//
// When the robot is stationary (vx=vy=omega=0), p_target = hip_pos and the
// foot cycles straight up and back down in place (idle animation).
// ---------------------------------------------------------------------------
static vec3_t raibert_target(float vx, float vy, float omega, int leg,
                             float beta, float freq) {
  float T_stance = beta / freq;

  float vx_hip = vx - omega * HIP_Y[leg];
  float vy_hip = vy + omega * HIP_X[leg];

  return (vec3_t){
      .x = HIP_X[leg] + vx_hip * (T_stance * 0.5f),
      .y = HIP_Y[leg] + vy_hip * (T_stance * 0.5f),
      .z = -BODY_HEIGHT,
  };
}

// ---------------------------------------------------------------------------
// gait_init
//
// Sets up trot defaults and places each foot at its rest position: directly
// below its hip socket at the nominal body height.
//
// master_phase starts at 0.  With the offsets above that means FL/RR are
// right at the start of their stance phase and FR/RL are right at the start
// of their swing phase.  On the first tick FR/RL will detect a liftoff
// transition, record p_lift = rest position, compute p_target = rest position
// (since v=0 initially), and swing straight up and back down.  Clean start.
// ---------------------------------------------------------------------------
void gait_init(gait_t *g) {
  memset(g, 0, sizeof(*g));

  g->beta = 0.5f; // trot: 50% stance
  g->freq = 1.5f; // 1.5 Hz oscillator ≈ comfortable walking pace
  g->master_phase = 0.0f;

  for (int i = 0; i < NUM_LEGS; i++) {
    vec3_t rest = {HIP_X[i], HIP_Y[i], -BODY_HEIGHT};
    g->legs[i].p_contact = rest;
    g->legs[i].p_lift = rest;
    g->legs[i].p_target = rest;
    g->legs[i].in_swing = false;
  }
}

// ---------------------------------------------------------------------------
// gait_update — the full per-tick update.
//
// Called at a fixed rate (dt seconds per call, e.g. 0.010 for 100 Hz).
// Advances the oscillator, runs stance/swing physics for each leg, and writes
// the resulting foot positions into foot_out[].
// ---------------------------------------------------------------------------
void gait_update(gait_t *g, float vx, float vy, float omega, float dt,
                 vec3_t foot_out[NUM_LEGS]) {

  // -----------------------------------------------------------------------
  // 1. Advance the master phase clock.
  //
  // master_phase increments by freq*dt each tick.  At freq=1.5 Hz and
  // dt=0.010 s that's 0.015 per tick, so one full cycle takes 1/1.5 = 0.667 s.
  //
  // We subtract 1.0 instead of using fmodf to avoid a function call in the
  // tight loop — the increment is always < 1.0 so one subtraction suffices.
  // -----------------------------------------------------------------------
  g->master_phase += g->freq * dt;
  if (g->master_phase >= 1.0f)
    g->master_phase -= 1.0f;

  // -----------------------------------------------------------------------
  // 2. Pre-compute yaw rotation coefficients for stance foot tracking.
  //
  // When the body yaws by omega*dt (CCW = positive = turning left), every
  // contact point that is fixed to the ground appears to rotate by -omega*dt
  // in the body frame.
  //
  // 2D rotation by angle θ = -omega*dt:
  //   x' =  x*cos(θ) - y*sin(θ) = x*cos(ω*dt) + y*sin(ω*dt)
  //   y' =  x*sin(θ) + y*cos(θ) = -x*sin(ω*dt) + y*cos(ω*dt)
  //
  // We compute c/s once here and reuse across all legs.
  // -----------------------------------------------------------------------
  float c_yaw = cosf(omega * dt);
  float s_yaw = sinf(omega * dt);

  for (int i = 0; i < NUM_LEGS; i++) {
    gait_leg_t *leg = &g->legs[i];

    // -------------------------------------------------------------------
    // 3. Compute this leg's effective phase and swing state.
    //
    // phi is the per-leg phase after applying the fixed offset.
    // fmodf keeps it in [0, 1) in case the sum exceeds 1.
    //
    // phi < beta  → stance  (foot on ground, body moving over it)
    // phi >= beta → swing   (foot in the air, tracing an arc)
    // -------------------------------------------------------------------
    float phi = fmodf(g->master_phase + PHASE_OFFSET[i], 1.0f);
    bool was_swing = leg->in_swing;
    bool now_swing = (phi >= g->beta);
    leg->in_swing = now_swing;

    // -------------------------------------------------------------------
    // 4. Detect state transitions.
    //
    // TOUCHDOWN (swing → stance):
    //   Accept p_target as the new contact point.  The swing arc ends
    //   exactly at p_target (s=1 → lerp returns p_target), so this is
    //   a continuity-preserving snap, not a jump.
    //
    // LIFTOFF (stance → swing):
    //   Snapshot where the foot currently is (p_lift) so the swing arc
    //   has a well-defined start point.  Then compute the landing target
    //   using the Raibert heuristic with the current velocity command.
    //   We compute p_target here, at liftoff, and hold it fixed through
    //   the entire swing.  This matches how Raibert's original controller
    //   works: commit to a landing site at the start of swing, don't
    //   continuously re-plan mid-air.
    // -------------------------------------------------------------------
    if (was_swing && !now_swing) {
      leg->p_contact = leg->p_target;
    }

    if (!was_swing && now_swing) {
      leg->p_lift = leg->p_contact;
      leg->p_target = raibert_target(vx, vy, omega, i, g->beta, g->freq);
    }

    // -------------------------------------------------------------------
    // 5a. STANCE — track the contact point in body frame.
    //
    // The foot is fixed to the world.  The body is moving, so from the
    // body's perspective the contact point drifts backward and sideways.
    //
    // Translation drift:
    //   p -= v * dt   (body moves forward → foot appears to move back)
    //
    // Yaw drift:
    //   Rotate p by -omega*dt using the pre-computed c_yaw / s_yaw.
    //
    // We apply translation first, then yaw — order matters for large dt
    // but at 100 Hz the error is negligible (second-order in dt).
    //
    // z is clamped to -BODY_HEIGHT: the foot is on a flat ground plane
    // and the body height is constant for now.
    // -------------------------------------------------------------------
    if (!now_swing) {
      leg->p_contact.x -= vx * dt;
      leg->p_contact.y -= vy * dt;

      float cx = leg->p_contact.x;
      float cy = leg->p_contact.y;
      leg->p_contact.x = c_yaw * cx + s_yaw * cy;
      leg->p_contact.y = -s_yaw * cx + c_yaw * cy;
      leg->p_contact.z = -BODY_HEIGHT;

      foot_out[i] = leg->p_contact;

      // -------------------------------------------------------------------
      // 5b. SWING — trace a smooth arc from p_lift to p_target.
      //
      // s is the normalised swing phase ∈ [0, 1]:
      //   s = (phi - beta) / (1 - beta)
      //   s = 0 at liftoff, s = 1 at touchdown.
      //
      // XY: linear interpolation from lift point to landing target.
      //   Simple lerp is smooth enough for the foot path in the XY plane.
      //   The body can't see the foot during swing anyway — no visual reason
      //   to use a fancier curve.
      //
      // Z: sinusoidal arc.
      //   z = -BODY_HEIGHT + STEP_HEIGHT * sin(π * s)
      //   At s=0 and s=1 the foot is at ground level.
      //   At s=0.5 (mid-swing) the foot is at peak height STEP_HEIGHT above
      //   the ground.  sin(π*s) is C¹ at both endpoints (derivative = 0
      //   where it meets the stance phase), which avoids foot slapping.
      // -------------------------------------------------------------------
    } else {
      float s = (phi - g->beta) / (1.0f - g->beta);

      foot_out[i] = (vec3_t){
          .x = lerp(leg->p_lift.x, leg->p_target.x, s),
          .y = lerp(leg->p_lift.y, leg->p_target.y, s),
          .z = -BODY_HEIGHT + STEP_HEIGHT * sinf((float)M_PI * s),
      };
    }
  }
}
