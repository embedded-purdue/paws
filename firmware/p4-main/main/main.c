#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gait_task.h"
#include "mailbox.h"
#include "uart_rx.h"

static const char *TAG = "paws_p4";

// Shared mailbox — UART RX task writes, gait task reads.
static gamepad_mailbox_t s_mailbox;

void app_main(void) {
  ESP_LOGI(TAG, "PAWS P4 starting");

  mailbox_init(&s_mailbox);

  // UART RX: CPU0, priority 5 — receives raw gamepad frames from S3.
  uart_rx_start(&s_mailbox);

  // Gait + IK: CPU1, priority 6 — 100 Hz oscillator + future IK.
  gait_task_start(&s_mailbox);

  ESP_LOGI(TAG, "tasks running");

  // app_main can return; FreeRTOS keeps the tasks alive.
}
