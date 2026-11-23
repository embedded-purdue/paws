#pragma once

// ---------------------------------------------------------------------------
// uart_rx  —  UART frame receiver task for the ESP32-P4.
//
// Runs as a FreeRTOS task pinned to CPU0.  Opens UART1
// (RX = GPIO 38, TX = GPIO 37, 921600 baud), reads 21-byte wire frames,
// validates CRC-8/SMBUS, and writes good frames to the shared mailbox.
//
// Frame format: [0xA5][0x5A][18-byte payload][CRC8]  (defined in protocol.h)
// ---------------------------------------------------------------------------

#include "mailbox.h"

// uart_rx_start — initialise UART1 and launch the RX task on CPU0.
//
//   mailbox — shared mailbox written by this task, read by gait_task.
void uart_rx_start(gamepad_mailbox_t *mailbox);
