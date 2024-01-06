// Copyright (c) 2024, Adam Simpkins
#include "ausb/hw/esp/TaskNotificationLoop.h"

namespace ausb {

void TaskNotificationLoop::init() {
  return init(xTaskGetCurrentTaskHandle());
}

bool TaskNotificationLoop::wait(std::chrono::milliseconds timeout) {
  assert(xTaskGetCurrentTaskHandle() == task_);

  uint32_t const bits_to_clear_on_entry = 0;
  uint32_t const bits_to_clear_on_exit = 0xffffffff;
  uint32_t value;
  TickType_t ticks_to_wait = pdMS_TO_TICKS(timeout.count());
  auto ret = xTaskNotifyWait(
      bits_to_clear_on_entry, bits_to_clear_on_exit, &value, ticks_to_wait);
  // Returns pdTRUE if a notification was received, or pdFALSE on timeout
  return (ret == pdTRUE);
}

void TaskNotificationLoop::wake_from_isr() {
  uint32_t value = 1;
  eNotifyAction action = eSetBits;
  BaseType_t higher_priority_task_woken = 0;
  xTaskNotifyFromISR(task_, value, action, &higher_priority_task_woken);
}

} // namespace ausb
