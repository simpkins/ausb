// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/dev/ControlHandler.h"
#include "ausb/dev/EndpointManager.h"
#include "ausb/hw/HWDevice.h"

namespace ausb::device {

/**
 * UsbDevice is the primary class for defining a USB device implementation.
 *
 * This class is mostly just a small glue layer that combines various other
 * components together into a full device implementation.  The other parts of
 * libausb are designed to be used directly if you want to provide your own
 * custom behavior for some component.  However, in most common cases if you
 * just want standard device behavior, UsbDevice ties all of the pieces
 * together for you.
 *
 * The UsbDeviceImpl template parameter should be a class that you provide
 * with the device implementation.  See the UsbDeviceExample class in
 * ausb/dev/UsbDeviceExample.h for documentation about the methods that this
 * class needs to provide.
 */
template<typename UsbDeviceImpl>
class UsbDevice : private ControlHandlerCallback {
public:
  constexpr UsbDevice() noexcept = default;

  /**
   * Initialize the USB device and connect to the bus.
   */
  std::error_code init() { return ep_manager_.init(); }

  /**
   * Run the main USB task loop, processing events from the hardware.
   *
   * This just loops calling wait_for_event() followed by handle_event().
   * If desired you can implement your own custom task loop instead if you want
   * to perform any additional work.
   */
  void loop() { ep_manager_.loop(); }

  /**
   * Wait for a USB event to process.
   */
  DeviceEvent wait_for_event(std::chrono::milliseconds timeout) {
    return hw_.wait_for_event(timeout);
  }
  /**
   * Process one USB event.
   */
  void handle_event(const DeviceEvent &event) {
    ep_manager_.handle_event(event);
  }

  HWDevice *hw() { return &hw_; }

  /*
   * Get the device's descriptor map.
   */
  static constexpr const auto &descriptor_map() { return descriptors_; }

  UsbDeviceImpl &dev() { return *impl_; }

private:
  UsbDevice(UsbDevice const &) = delete;
  UsbDevice &operator=(UsbDevice const &) = delete;

  bool set_configuration(uint8_t config_id) override {
    return impl_.set_configuration(config_id, ep_manager_);
  }
  std::optional<buf_view> get_descriptor(uint16_t value,
                                         uint16_t index) override {
    return descriptors_.get_descriptor_with_setup_ids(value, index);
  }

  static constexpr auto descriptors_ = UsbDeviceImpl::make_descriptor_map();

  HWDevice hw_;
  ControlHandler ctrl_handler{this};
  EndpointManager ep_manager_{&hw_, &ctrl_handler};
  UsbDeviceImpl impl_;
};

} // namespace ausb::device
