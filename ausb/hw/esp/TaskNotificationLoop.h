// Copyright (c) 2024, Adam Simpkins
#pragma once

#include "ausb/hw/esp/EspTaskLoop.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <chrono>
#include <cinttypes>

namespace ausb {

/**
 * An EspTaskLoop implemented using "Direct to Task" notifications (the
 * xTaskNotifyWait() APIs).
 */
class TaskNotificationLoop : public EspTaskLoop {
public:
  /**
   * Wake flag bits.
   *
   * We currently only define one bit used for wake_from_usb_isr().
   * Users may use set other wake bits for their own purposes.
   */
  enum WakeFlags : uint32_t { WakeUsb = 0x00000001 };

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
   * Wait until the task has been woken up.
   *
   * This may only be called from the task specified in init().
   *
   * Returns the flag bits set by the wake*() calls, or 0 if a timeout
   * occurred.
   *
   * Note that if you call wake_from_task() or wake_from_isr() with flags set
   * to 0 this will cause a wake-up event but flags returned will still be 0 if
   * no other wake flags have been set by separate wake calls.
   */
  uint32_t wait(std::chrono::milliseconds timeout);

  /**
   * Wait with no timeout.
   */
  uint32_t wait_forever();

  /**
   * Cause any pending wait() call to wake up.
   *
   * If the task is not currently calling wait(), the next call to wait() will
   * return immediately.
   *
   * The flags passed in will be bitwise ORed with any existing wake flags.
   *
   * This version of wake may not be called from interrupt service routines.
   */
  void wake_from_task(uint32_t flags);

  /**
   * Cause any pending wait() call to wake up.
   *
   * This version of wake may be called from interrupt service routines.
   */
  void wake_from_isr(uint32_t flags);

  void wake_from_usb_isr() override final {
    wake_from_isr(WakeUsb);
  }

private:
  // Forbidden copy constructor and assignment operator
  TaskNotificationLoop(TaskNotificationLoop const &) = delete;
  TaskNotificationLoop &operator=(TaskNotificationLoop const &) = delete;

  TaskHandle_t task_ = nullptr;
};

} // namespace ausb
