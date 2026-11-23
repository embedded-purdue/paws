#include "uart_rx.h"

#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "protocol.h"

// ---------------------------------------------------------------------------
// Hardware configuration
// P4 RX = GPIO38 (← S3 TX GPIO17), P4 TX = GPIO37 (→ S3 RX GPIO18)
// ---------------------------------------------------------------------------
#define RX_UART_PORT UART_NUM_1
#define RX_UART_BAUD 921600
// P4 RX = GPIO4 (← S3 TX GPIO17), P4 TX = GPIO5 (→ S3 RX GPIO18)
#define RX_PIN 4
#define TX_PIN 5

// RX ring buffer: 8 frames of headroom.
#define RX_BUF (PROTO_FRAME_LEN * 8)
#define TX_BUF 0 // we never transmit on this port

#define RX_TASK_STACK 3072
#define RX_TASK_PRIO 5
#define RX_TASK_CORE 0

// Timeout for uart_read_bytes: long enough to never starve the loop.
// At 100 Hz frames, 50 ms = 5 missed frames before we'd time out.
#define READ_TIMEOUT_MS 50
#define READ_TICKS pdMS_TO_TICKS(READ_TIMEOUT_MS)

static const char *TAG = "uart_rx";

// ---------------------------------------------------------------------------
// read_byte — read exactly one byte; returns false on timeout or error.
// ---------------------------------------------------------------------------
static inline bool read_byte(uint8_t *b) {
  return uart_read_bytes(RX_UART_PORT, b, 1, READ_TICKS) == 1;
}

// ---------------------------------------------------------------------------
// rx_task — sync search → payload read → CRC validate → mailbox write.
//
// Sync search: scan byte-by-byte for 0xA5.  On a hit, peek the next byte
// for 0x5A.  If instead we see another 0xA5 we may still be in a sync
// sequence — check the following byte for 0x5A.  A CRC mismatch drops the
// frame and re-enters sync search, naturally handling 0xA5 0x5A patterns
// inside a payload.
// ---------------------------------------------------------------------------
static void rx_task(void *arg) {
  gamepad_mailbox_t *mailbox = (gamepad_mailbox_t *)arg;

  uint8_t b;
  uint8_t payload[PROTO_PAYLOAD_LEN];

  while (1) {
    // --- SYNC0 search ---
    if (!read_byte(&b))
      continue;
    if (b != PROTO_SYNC0)
      continue;

    // --- SYNC1 check ---
    if (!read_byte(&b))
      continue;
    if (b == PROTO_SYNC0) {
      // Could be start of a new frame; check the next byte for SYNC1.
      if (!read_byte(&b))
        continue;
    }
    if (b != PROTO_SYNC1)
      continue;

    // --- payload ---
    int got =
        uart_read_bytes(RX_UART_PORT, payload, PROTO_PAYLOAD_LEN, READ_TICKS);
    if (got != PROTO_PAYLOAD_LEN)
      continue;

    // --- CRC ---
    uint8_t crc_rx;
    if (!read_byte(&crc_rx))
      continue;

    uint8_t crc_calc = proto_crc8(payload, PROTO_PAYLOAD_LEN);
    if (crc_rx != crc_calc) {
      ESP_LOGW(TAG, "CRC mismatch: got 0x%02X want 0x%02X — resyncing", crc_rx,
               crc_calc);
      continue;
    }

    // --- write to mailbox ---
    gamepad_wire_t frame;
    memcpy(&frame, payload, sizeof(frame));
    mailbox_write(mailbox, &frame);
  }
}

// ---------------------------------------------------------------------------
// uart_rx_start
// ---------------------------------------------------------------------------
void uart_rx_start(gamepad_mailbox_t *mailbox) {
  uart_config_t cfg = {
      .baud_rate = RX_UART_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(uart_param_config(RX_UART_PORT, &cfg));
  ESP_ERROR_CHECK(uart_set_pin(RX_UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(
      uart_driver_install(RX_UART_PORT, RX_BUF, TX_BUF, 0, NULL, 0));

  ESP_LOGI(TAG, "UART1 ready: RX=GPIO%d TX=GPIO%d %d baud", RX_PIN, TX_PIN,
           RX_UART_BAUD);

  xTaskCreatePinnedToCore(rx_task, "uart_rx", RX_TASK_STACK, mailbox,
                          RX_TASK_PRIO, NULL, RX_TASK_CORE);
}
