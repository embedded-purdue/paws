#include "mailbox.h"

#include <string.h>

void mailbox_init(gamepad_mailbox_t *mb) {
  memset(&mb->state, 0, sizeof(mb->state));
  mb->valid = false;
  mb->mutex = xSemaphoreCreateMutex();
}

void mailbox_write(gamepad_mailbox_t *mb, const gamepad_wire_t *src) {
  xSemaphoreTake(mb->mutex, portMAX_DELAY);
  mb->state = *src;
  mb->valid = true;
  xSemaphoreGive(mb->mutex);
}

bool mailbox_read(gamepad_mailbox_t *mb, gamepad_wire_t *dst) {
  xSemaphoreTake(mb->mutex, portMAX_DELAY);
  bool v = mb->valid;
  if (v)
    *dst = mb->state;
  xSemaphoreGive(mb->mutex);
  return v;
}
