#include <stdlib.h>

#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include <uni.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <math.h>

#include "gait.h"
#include "imu.h"
#include "sdkconfig.h"

#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Set CONFIG_BLUEPAD32_PLATFORM_CUSTOM=y in sdkconfig.defaults"
#endif

typedef struct {
  int32_t axis_x;   // left  stick X  [-512, 511]
  int32_t axis_y;   // left  stick Y  [-512, 511]
  int32_t axis_rx;  // right stick X
  int32_t axis_ry;  // right stick Y
  int32_t brake;    // L2 / left trigger  [0, 1023]
  int32_t throttle; // R2 / right trigger [0, 1023]
  uint16_t buttons; // BUTTON_A … BUTTON_THUMB_R bitmask
  uint8_t dpad;     // DPAD_UP … DPAD_LEFT bitmask
  bool connected;
} gamepad_state_t;

static gamepad_state_t g_gamepad = {0};
static SemaphoreHandle_t g_gamepad_mutex;

typedef struct {
  uni_gamepad_seat_t seat;
} paws_instance_t;

static paws_instance_t *get_instance(uni_hid_device_t *d) {
  return (paws_instance_t *)&d->platform_data[0];
}

static void platform_init(int argc, const char **argv) {
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);
}

static void platform_on_init_complete(void) {
  uni_bt_start_scanning_and_autoconnect_unsafe();
  uni_bt_allow_incoming_connections(true);
  uni_bt_del_keys_unsafe();
}

static uni_error_t platform_on_device_discovered(bd_addr_t addr,
                                                 const char *name, uint16_t cod,
                                                 uint8_t rssi) {
  (void)addr;
  (void)name;
  (void)cod;
  (void)rssi;
  return UNI_ERROR_SUCCESS;
}

static void platform_on_device_connected(uni_hid_device_t *d) {
  logi("paws: controller connected %p\n", d);
}

static void platform_on_device_disconnected(uni_hid_device_t *d) {
  logi("paws: controller disconnected %p\n", d);
  xSemaphoreTake(g_gamepad_mutex, portMAX_DELAY);
  memset(&g_gamepad, 0, sizeof(g_gamepad)); // clear stale axes/buttons
  xSemaphoreGive(g_gamepad_mutex);
}

static uni_error_t platform_on_device_ready(uni_hid_device_t *d) {
  get_instance(d)->seat = GAMEPAD_SEAT_A;
  logi("paws: controller ready\n");
  return UNI_ERROR_SUCCESS;
}

static void platform_on_controller_data(uni_hid_device_t *d,
                                        uni_controller_t *ctl) {
  if (ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD)
    return;
  uni_gamepad_t *gp = &ctl->gamepad;

  xSemaphoreTake(g_gamepad_mutex, portMAX_DELAY);
  g_gamepad.axis_x = gp->axis_x;
  g_gamepad.axis_y = gp->axis_y;
  g_gamepad.axis_rx = gp->axis_rx;
  g_gamepad.axis_ry = gp->axis_ry;
  g_gamepad.brake = gp->brake;
  g_gamepad.throttle = gp->throttle;
  g_gamepad.buttons = gp->buttons;
  g_gamepad.dpad = gp->dpad;
  g_gamepad.connected = true;
  xSemaphoreGive(g_gamepad_mutex);
}

static const uni_property_t *platform_get_property(uni_property_idx_t idx) {
  ARG_UNUSED(idx);
  return NULL;
}

static void platform_on_oob_event(uni_platform_oob_event_t event, void *data) {
  if (event == UNI_PLATFORM_OOB_BLUETOOTH_ENABLED)
    logi("paws: BT scanning %s\n",
         (bool)(intptr_t)data ? "started" : "stopped");
}

static struct uni_platform *get_platform(void) {
  static struct uni_platform plat = {
      .name = "paws",
      .init = platform_init,
      .on_init_complete = platform_on_init_complete,
      .on_device_discovered = platform_on_device_discovered,
      .on_device_connected = platform_on_device_connected,
      .on_device_disconnected = platform_on_device_disconnected,
      .on_device_ready = platform_on_device_ready,
      .on_controller_data = platform_on_controller_data,
      .get_property = platform_get_property,
      .on_oob_event = platform_on_oob_event,
  };
  return &plat;
}

// ---------------------------------------------------------------------------
// Gait task — runs at 100 Hz on CPU1.
//
// Responsibilities:
//   1. Snapshot the gamepad state (mutex-protected, written by CPU0 BT
//   callback).
//   2. Convert raw joystick axes to physical velocity commands.
//   3. Call gait_update() to advance the oscillator.
//   4. foot_out[] holds the 4 foot targets; IK goes here later.
//
// Joystick axes from Bluepad32 / Xbox:
//   axis_y  — left stick vertical.   Pushing forward is typically negative.
//             Negate → positive vx = move forward.
//   axis_x  — left stick horizontal. Pushing right is positive.
//             Negate → positive vy = move left (body frame y = left).
//   axis_rx — right stick horizontal. Pushing right is positive.
//             Negate → positive omega = turn left (z-up right-hand rule).
//
// Deadzone: raw values below DZ_THRESHOLD (out of ±511) are zeroed.
// Scale: divide by 511.0 to normalise to [−1, 1], then multiply by max speed.
//
// These are reasonable starting values — tune MAX_VX/VY/OMEGA at runtime.
// ---------------------------------------------------------------------------
#define GAIT_DT_MS 10      // task period [ms] = 100 Hz
#define GAIT_DT_S 0.010f   // same in seconds, used by gait_update
#define DZ_THRESHOLD 25.0f // joystick deadzone — measured resting bias is ±13
#define MAX_VX 0.5f        // max forward speed    [m/s]
#define MAX_VY 0.3f        // max lateral speed    [m/s]
#define MAX_OMEGA 1.5f     // max yaw rate         [rad/s]

static void gait_task(void *arg) {
  (void)arg;

  gait_t gait;
  vec3_t feet[NUM_LEGS];
  gait_init(&gait);

  TickType_t wake = xTaskGetTickCount();

  while (1) {
    // -- 1. snapshot gamepad ----------------------------------------
    gamepad_state_t gp;
    xSemaphoreTake(g_gamepad_mutex, portMAX_DELAY);
    gp = g_gamepad;
    xSemaphoreGive(g_gamepad_mutex);

    // -- 2. joystick → velocity commands ----------------------------
    // Apply deadzone then normalise and scale.
    // axis_y / axis_x / axis_rx range: [-512, 511]
    float raw_ly = (float)gp.axis_y;
    float raw_lx = (float)gp.axis_x;
    float raw_rx = (float)gp.axis_rx;

    float vx =
        (fabsf(raw_ly) > DZ_THRESHOLD) ? -(raw_ly / 511.0f) * MAX_VX : 0.0f;
    float vy =
        (fabsf(raw_lx) > DZ_THRESHOLD) ? -(raw_lx / 511.0f) * MAX_VY : 0.0f;
    float omega =
        (fabsf(raw_rx) > DZ_THRESHOLD) ? -(raw_rx / 511.0f) * MAX_OMEGA : 0.0f;

    // -- 3. advance gait oscillator ---------------------------------
    gait_update(&gait, vx, vy, omega, GAIT_DT_S, feet);

    // -- 4. Print at 10 Hz: raw sticks | velocity commands | leg S/W + xyz ---
    // S = stance (foot on ground), W = swing (foot in air).
    // Showing raw stick values alongside velocity makes it easy to see whether
    // the deadzone is eating input or the sign convention is flipped.
    static uint32_t tick = 0;
    if (++tick % 10 == 0) {
      printf("lx=%4d ly=%4d rx=%4d | vx=%+.2f vy=%+.2f w=%+.2f | "
             "FL:%c(%+.3f,%+.3f,%+.3f) "
             "FR:%c(%+.3f,%+.3f,%+.3f) "
             "RL:%c(%+.3f,%+.3f,%+.3f) "
             "RR:%c(%+.3f,%+.3f,%+.3f)\n",
             (int)gp.axis_x, (int)gp.axis_y, (int)gp.axis_rx, vx, vy, omega,
             gait.legs[LEG_FL].in_swing ? 'W' : 'S', feet[LEG_FL].x,
             feet[LEG_FL].y, feet[LEG_FL].z,
             gait.legs[LEG_FR].in_swing ? 'W' : 'S', feet[LEG_FR].x,
             feet[LEG_FR].y, feet[LEG_FR].z,
             gait.legs[LEG_RL].in_swing ? 'W' : 'S', feet[LEG_RL].x,
             feet[LEG_RL].y, feet[LEG_RL].z,
             gait.legs[LEG_RR].in_swing ? 'W' : 'S', feet[LEG_RR].x,
             feet[LEG_RR].y, feet[LEG_RR].z);
    }

    // -- 5. sleep until next period ---------------------------------
    // vTaskDelayUntil sleeps for exactly GAIT_DT_MS regardless of how
    // long steps 1-4 took, keeping the loop period deterministic.
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(GAIT_DT_MS));
  }
}

int app_main(void) {
  g_gamepad_mutex = xSemaphoreCreateMutex();
  imu_start();

  // Gait oscillator on CPU1 (same core as BNO085, higher priority so it
  // always runs on schedule regardless of sensor poll timing).
  xTaskCreatePinnedToCore(gait_task, "gait", 4096, NULL,
                          /*priority*/ 6, NULL, /*core*/ 1);

  // BTstack stdio — skip when USB console is active (they conflict)
#ifdef CONFIG_ESP_CONSOLE_UART
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
  btstack_stdio_init();
#endif
#endif

  btstack_init();
  uni_platform_set_custom(get_platform());
  uni_init(0, NULL);

  btstack_run_loop_execute(); // never returns; BT lives here on CPU0
  return 0;
}
