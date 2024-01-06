// Copyright (c) 2024, Adam Simpkins
#pragma once

#include "ausb/hw/esp/EspTaskLoop.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <chrono>

namespace ausb {

/**
 * An EspTaskLoop implemented using "Direct to Task" notifications (the
 * xTaskNotifyWait() APIs).
 */
class TaskNotificationLoop : public EspTaskLoop {
public:
  constexpr TaskNotificationLoop() = default;

  /**
   * Initialize the TaskNotificationLoop.
   *
   * This version of init() must be called from the task that will call wait().
   */
  void init();

  /**
   * Initialize the TaskNotificationLoop.
   */
  void init(TaskHandle_t task) {
    task_ = task;
  }

  /**
   * Wait until wake_from_isr() has been called.
   *
   * Returns true if the wait was woken by an event, or false if a timeout
   * occurred.
   */
  bool wait(std::chrono::milliseconds timeout);

  void wake_from_isr() override final;

private:
  // Forbidden copy constructor and assignment operator
  TaskNotificationLoop(TaskNotificationLoop const &) = delete;
  TaskNotificationLoop &operator=(TaskNotificationLoop const &) = delete;

  TaskHandle_t task_ = nullptr;
};

} // namespace ausb
