#include "gait_task.h"

#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gait.h"

// ---------------------------------------------------------------------------
// Velocity scaling — mirrors s3-bluetooth/main/main.c.
// All tuning lives here; the S3 is a dumb relay.
// ---------------------------------------------------------------------------
#define DZ_THRESHOLD 25.0f // joystick deadzone (resting bias is ±13)
#define MAX_VX 0.5f        // max forward speed    [m/s]
#define MAX_VY 0.3f        // max lateral speed    [m/s]
#define MAX_OMEGA 1.5f     // max yaw rate         [rad/s]

#define GAIT_DT_MS 10    // 100 Hz period [ms]
#define GAIT_DT_S 0.010f // same in seconds

static inline float apply_dz(float raw, float scale) {
  return (fabsf(raw) > DZ_THRESHOLD) ? (raw / 511.0f) * scale : 0.0f;
}

static void gait_task(void *arg) {
  gamepad_mailbox_t *mailbox = (gamepad_mailbox_t *)arg;

  gait_t gait;
  vec3_t feet[NUM_LEGS];
  gait_init(&gait);

  TickType_t wake = xTaskGetTickCount();
  uint32_t tick = 0;

  while (1) {
    // -- 1. Snapshot mailbox ------------------------------------------
    gamepad_wire_t gp = {0};
    bool have_ctrl = mailbox_read(mailbox, &gp);

    // -- 2. Axes → velocity commands ----------------------------------
    // Negate Y: push forward (negative axis) → positive vx.
    // Negate X: push right (positive axis)   → body-frame vy is left.
    // Negate RX: push right (positive axis)  → turn right = negative ω.
    float vx = have_ctrl ? -apply_dz((float)gp.axis_y, MAX_VX) : 0.0f;
    float vy = have_ctrl ? -apply_dz((float)gp.axis_x, MAX_VY) : 0.0f;
    float omega = have_ctrl ? -apply_dz((float)gp.axis_rx, MAX_OMEGA) : 0.0f;

    // -- 3. Advance gait oscillator ------------------------------------
    gait_update(&gait, vx, vy, omega, GAIT_DT_S, feet);

    // -- 4. IK placeholder --------------------------------------------
    // TODO: convert feet[] body-frame targets to joint angles,
    //       send to servo bus.

    // -- 5. Debug print at 10 Hz (every 10 ticks) ---------------------
    if (++tick % 10 == 0) {
      printf("ctrl=%d lx=%4d ly=%4d rx=%4d | "
             "vx=%+.2f vy=%+.2f w=%+.2f | "
             "FL:%c(%+.3f,%+.3f,%+.3f) "
             "FR:%c(%+.3f,%+.3f,%+.3f) "
             "RL:%c(%+.3f,%+.3f,%+.3f) "
             "RR:%c(%+.3f,%+.3f,%+.3f)\n",
             gp.connected, (int)gp.axis_x, (int)gp.axis_y, (int)gp.axis_rx, vx,
             vy, omega, gait.legs[LEG_FL].in_swing ? 'W' : 'S', feet[LEG_FL].x,
             feet[LEG_FL].y, feet[LEG_FL].z,
             gait.legs[LEG_FR].in_swing ? 'W' : 'S', feet[LEG_FR].x,
             feet[LEG_FR].y, feet[LEG_FR].z,
             gait.legs[LEG_RL].in_swing ? 'W' : 'S', feet[LEG_RL].x,
             feet[LEG_RL].y, feet[LEG_RL].z,
             gait.legs[LEG_RR].in_swing ? 'W' : 'S', feet[LEG_RR].x,
             feet[LEG_RR].y, feet[LEG_RR].z);
    }

    // -- 6. Sleep until next 100 Hz boundary --------------------------
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(GAIT_DT_MS));
  }
}

void gait_task_start(gamepad_mailbox_t *mailbox) {
  xTaskCreatePinnedToCore(gait_task, "gait", 4096, mailbox,
                          /*priority*/ 6, NULL, /*core*/ 1);
}
