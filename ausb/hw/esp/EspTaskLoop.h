// Copyright (c) 2024, Adam Simpkins
#pragma once

namespace ausb {

/**
 * A virtual API for allowing the USB interrupt handlers to wake the task
 * responsible for processing USB events.
 *
 * This allows users to provide their own task loop and inform the AUSB
 * interrupt handlers how to wake it up when a USB event is ready.
 *
 * Some possible task loop implementations could include:
 * - FreeRTOS queues
 * - ESP-IDF esp_event loops
 * - Direct to Task Notifications (xTaskNotify)
 * - Stream buffers
 */
class EspTaskLoop {
public:
  constexpr EspTaskLoop() noexcept = default;

  virtual void wake_from_usb_isr() = 0;

private:
  EspTaskLoop(EspTaskLoop const &) = delete;
  EspTaskLoop &operator=(EspTaskLoop const &) = delete;
};

} // namespace ausb
