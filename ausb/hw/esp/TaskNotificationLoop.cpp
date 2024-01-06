// Copyright (c) 2024, Adam Simpkins
#include "ausb/hw/esp/TaskNotificationLoop.h"

namespace ausb {

void TaskNotificationLoop::init() {
  return init(xTaskGetCurrentTaskHandle());
}

uint32_t TaskNotificationLoop::wait(std::chrono::milliseconds timeout) {
  assert(xTaskGetCurrentTaskHandle() == task_);

  uint32_t const bits_to_clear_on_entry = 0;
  uint32_t const bits_to_clear_on_exit = 0xffffffff;
  uint32_t value;
  TickType_t const ticks_to_wait = pdMS_TO_TICKS(timeout.count());
  auto const ret = xTaskNotifyWait(
      bits_to_clear_on_entry, bits_to_clear_on_exit, &value, ticks_to_wait);
  // Returns pdTRUE if a notification was received, or pdFALSE on timeout
  return (ret == pdTRUE) ? value : 0;
}

uint32_t TaskNotificationLoop::wait_forever() {
  assert(xTaskGetCurrentTaskHandle() == task_);

  uint32_t const bits_to_clear_on_entry = 0;
  uint32_t const bits_to_clear_on_exit = 0xffffffff;
  TickType_t const ticks_to_wait = std::numeric_limits<TickType_t>::max();
  uint32_t value;
  while (true) {
    auto const ret = xTaskNotifyWait(
        bits_to_clear_on_entry, bits_to_clear_on_exit, &value, ticks_to_wait);
    if (ret == pdTRUE) {
      return value;
    }
  }
}

void TaskNotificationLoop::wake_from_task(uint32_t flags) {
  xTaskNotify(task_, flags, eSetBits);
}

void TaskNotificationLoop::wake_from_isr(uint32_t flags) {
  BaseType_t higher_priority_task_woken = 0;
  xTaskNotifyFromISR(task_, flags, eSetBits, &higher_priority_task_woken);
  if (higher_priority_task_woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

} // namespace ausb
