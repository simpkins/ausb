// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/dev/EndpointManager.h"
#include "ausb/dev/StdControlHandler.h"
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
template <typename UsbDeviceImpl, typename HwDeviceType = HWDevice>
class UsbDevice : private StdControlHandlerCallback {
public:
  constexpr UsbDevice() noexcept = default;

  /**
   * Initialize the USB device and connect to the bus.
   */
  template <typename... Args>
  std::error_code init(Args... args) {
    return ep_manager_.init(asel::forward<Args>(args)...);
  }

  HwDeviceType *hw() {
    return &hw_;
  }
  EndpointManager *manager() {
    return &ep_manager_;
  }

  /*
   * Get the device's descriptor map.
   */
  static constexpr const auto &descriptor_map() {
    return descriptors_;
  }

  UsbDeviceImpl &dev() {
    return impl_;
  }

private:
  UsbDevice(UsbDevice const &) = delete;
  UsbDevice &operator=(UsbDevice const &) = delete;

  bool set_configuration(uint8_t config_id) override {
    return impl_.set_configuration(config_id, ep_manager_);
  }
  std::optional<asel::buf_view> get_descriptor(uint16_t value,
                                               uint16_t index) override {
    return descriptors_.get_descriptor_with_setup_ids(value, index);
  }

  static constexpr auto descriptors_ = UsbDeviceImpl::make_descriptor_map();

  HwDeviceType hw_;
  StdControlHandler ctrl_handler{this};
  EndpointManager ep_manager_{&hw_, &ctrl_handler};
  UsbDeviceImpl impl_{&ep_manager_};
};

} // namespace ausb::device
