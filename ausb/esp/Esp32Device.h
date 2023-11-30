// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/DeviceEvent.h"
#include "ausb/usb_types.h"

#include <esp_err.h>
#include <esp_private/usb_phy.h>
#include <hal/gpio_types.h>
#include <soc/usb_struct.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>

namespace ausb {

enum class EspPhyType {
  Internal,
  External,
};

/**
 * A USB device implementation for ESP32-S2 and ESP32-S3 chips.
 *
 * Note that the Espressif documentation for USB functionality is somewhat
 * sparse.  The ESP32-S3 Technical Reference Manual indicates that full
 * register documentation is subject to an NDA, and therefore isn't easily
 * available.  The Technical Reference Manual contains a pretty high-level
 * overview of the behavior, and the comments in
 * esp-idf/components/soc/esp32s3/include/soc/usb_reg.h provide some minimal
 * descriptions of some of the register behavior.
 */
class Esp32Device {
public:
  explicit constexpr Esp32Device(usb_dev_t *usb = &USB0) noexcept : usb_{usb} {}
  ~Esp32Device();

  /**
   * Initialize the USB device.
   *
   * If the device is self-powered, the vbus_monitor parameter should be set
   * to a GPIO that is monitoring whether the voltage on the VBUS line is
   * present or not.  This allows the device to tell whether the bus is
   * connected or not.  Note that ESP32 pins are not 5V tolerant, so a voltage
   * divider must be used.  For bus-powered devices, the vbus_monitor parameter
   * may be set to std::nullopt.
   */
  [[nodiscard]] esp_err_t
  init(EspPhyType phy_type = EspPhyType::Internal,
       std::optional<gpio_num_t> vbus_monitor = std::nullopt);

  /**
   * Reset and de-initialize the USB device.
   */
  void reset();

  /**
   * Wait for a USB event.
   *
   * Returns the USB event, or UninitializedEvent if the timeout expires before
   * an event is ready.
   */
  DeviceEvent wait_for_event(std::chrono::milliseconds timeout);

  /**
   * Get the FreeRTOS event queue used to receive USB events.
   *
   * This can be used if the caller wishes to manually fetch events, rather
   * than using the wait_for_event() function.
   */
  QueueHandle_t event_queue() const { return event_queue_; }

private:
  // Speed bits used by the dcfg and dsts registers.
  // These unfortunately are not defined in soc/usb_reg.h
  enum Speed : uint32_t {
    High30Mhz = 0,
    Full30Mhz = 1,
    Low6Mhz = 2,
    Full48Mhz = 3,
  };

  Esp32Device(Esp32Device const &) = delete;
  Esp32Device &operator=(Esp32Device const &) = delete;

  void send_event_from_isr(const DeviceEvent& event);
  template <typename T, typename... Args>
  void send_event_from_isr(Args &&... args) {
    DeviceEvent e{std::in_place_type<T>, std::forward<Args>(args)...};
    send_event_from_isr(e);
  }

  [[nodiscard]] esp_err_t init_phy(EspPhyType phy_type,
                                   std::optional<gpio_num_t> vbus_monitor);
  void all_endpoints_nak();
  [[nodiscard]] esp_err_t enable_interrupts();
  static void static_interrupt_handler(void *arg);
  void interrupt_handler();

  void bus_reset();
  void enum_done();
  void rx_fifo_nonempty();
  void receive_packet(uint8_t endpoint_num, uint16_t packet_size);
  void handle_out_ep_interrupt();
  void handle_in_ep_interrupt();
  void out_endpoint_interrupt(uint8_t epnum);
  void in_endpoint_interrupt(uint8_t epnum);

  static constexpr uint8_t kMaxEventQueueSize = 32;

  StaticQueue_t queue_storage_ = {};
  QueueHandle_t event_queue_ = nullptr;
  usb_dev_t *usb_ = nullptr;
  usb_phy_handle_t phy_ = nullptr;
  intr_handle_t interrupt_handle_ = nullptr;
  std::array<uint8_t, sizeof(DeviceEvent) *kMaxEventQueueSize> queue_buffer_ =
      {};

  union {
    std::array<uint32_t, 2> u32;
    SetupPacket setup;
  } setup_packet_ = {};
};

} // namespace ausb
