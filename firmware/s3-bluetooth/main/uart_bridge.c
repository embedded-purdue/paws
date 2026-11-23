#include "uart_bridge.h"

#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "protocol.h"

// ---------------------------------------------------------------------------
// Hardware configuration
// ---------------------------------------------------------------------------

#define BRIDGE_UART_PORT UART_NUM_1
#define BRIDGE_UART_BAUD 921600
#define BRIDGE_TX_PIN 17
#define BRIDGE_RX_PIN 18

// IDF 6.x uart_driver_install requires rx_buffer_size > UART_HW_FIFO_LEN (128).
// There is no zero bypass in this version. 256 satisfies the check; the buffer
// sits unused since we never receive on this port.
#define BRIDGE_TX_BUF 256
#define BRIDGE_RX_BUF 256

// Task parameters: CPU0 (alongside BT), priority below gait (6) and BT.
#define BRIDGE_TASK_STACK 2048
#define BRIDGE_TASK_PRIO 4
#define BRIDGE_TASK_CORE 0

#define BRIDGE_DT_MS 10 // 100 Hz

static const char *TAG = "uart_bridge";

// ---------------------------------------------------------------------------
// Module state  —  set once by uart_bridge_start, read by the task.
// ---------------------------------------------------------------------------
static SemaphoreHandle_t s_mutex;
static const gamepad_state_t *s_state;

// ---------------------------------------------------------------------------
// pack_frame — build the 21-byte wire frame into `buf`.
//
// Converts in-memory gamepad_state_t (int fields) to the compact
// gamepad_wire_t (int16_t / uint16_t) and wraps with sync + CRC.
// ---------------------------------------------------------------------------
static void pack_frame(const gamepad_state_t *gp,
                       uint8_t buf[PROTO_FRAME_LEN]) {
  gamepad_wire_t wire;
  wire.axis_x = (int16_t)gp->axis_x;
  wire.axis_y = (int16_t)gp->axis_y;
  wire.axis_rx = (int16_t)gp->axis_rx;
  wire.axis_ry = (int16_t)gp->axis_ry;
  wire.brake = (uint16_t)gp->brake;
  wire.throttle = (uint16_t)gp->throttle;
  wire.buttons = gp->buttons;
  wire.dpad = gp->dpad;
  wire.connected = gp->connected ? 1u : 0u;

  buf[0] = PROTO_SYNC0;
  buf[1] = PROTO_SYNC1;
  memcpy(&buf[2], &wire, PROTO_PAYLOAD_LEN);
  buf[2 + PROTO_PAYLOAD_LEN] = proto_crc8(&buf[2], PROTO_PAYLOAD_LEN);
}

// ---------------------------------------------------------------------------
// bridge_task — 100 Hz TX loop on CPU0.
// ---------------------------------------------------------------------------
static void bridge_task(void *arg) {
  (void)arg;

  uint8_t frame[PROTO_FRAME_LEN];
  TickType_t wake = xTaskGetTickCount();

  while (1) {
    // Snapshot the shared gamepad state under the mutex.
    gamepad_state_t snap;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    snap = *s_state;
    xSemaphoreGive(s_mutex);

    // Build and send the wire frame.
    pack_frame(&snap, frame);
    uart_write_bytes(BRIDGE_UART_PORT, (const char *)frame, PROTO_FRAME_LEN);

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(BRIDGE_DT_MS));
  }
}

// ---------------------------------------------------------------------------
// uart_bridge_start
// ---------------------------------------------------------------------------
void uart_bridge_start(SemaphoreHandle_t mutex, const gamepad_state_t *state) {
  s_mutex = mutex;
  s_state = state;

  // Configure UART1: 921600 8N1, no flow control.
  uart_config_t cfg = {
      .baud_rate = BRIDGE_UART_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(uart_param_config(BRIDGE_UART_PORT, &cfg));
  ESP_ERROR_CHECK(uart_set_pin(BRIDGE_UART_PORT, BRIDGE_TX_PIN, BRIDGE_RX_PIN,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(BRIDGE_UART_PORT, BRIDGE_RX_BUF,
                                      BRIDGE_TX_BUF, 0, NULL, 0));

  ESP_LOGI(TAG, "UART1 ready: TX=GPIO%d RX=GPIO%d %d baud", BRIDGE_TX_PIN,
           BRIDGE_RX_PIN, BRIDGE_UART_BAUD);

  xTaskCreatePinnedToCore(bridge_task, "uart_bridge", BRIDGE_TASK_STACK, NULL,
                          BRIDGE_TASK_PRIO, NULL, BRIDGE_TASK_CORE);
}
