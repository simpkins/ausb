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
class UsbDevice {
public:
  constexpr UsbDevice() noexcept = default;

  std::error_code init() { return ep_manager_.init(); }
  void loop() { ep_manager_.loop(); }

  DeviceEvent wait_for_event(std::chrono::milliseconds timeout) {
    return hw_.wait_for_event(timeout);
  }
  void handle_event(const DeviceEvent &event) {
    ep_manager_.handle_event(event);
  }

  HWDevice *hw() { return &hw_; }

  /*
   * Get the device's descriptor map.
   */
  static constexpr const auto &descriptor_map() { return descriptors_; }

private:
  UsbDevice(UsbDevice const &) = delete;
  UsbDevice &operator=(UsbDevice const &) = delete;

  static constexpr auto descriptors_ = UsbDeviceImpl::make_descriptor_map();

  HWDevice hw_;
  ControlHandler ctrl_handler{&descriptors_};
  EndpointManager ep_manager_{&hw_, &ctrl_handler};
  UsbDeviceImpl impl_;
};

} // namespace ausb::device
