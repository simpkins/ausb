// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/UsbDevice.h"
#include "ausb/desc/StaticDescriptorMap.h"

namespace ausb::device::example {

/**
 * This class is purely an example that serves to document the API
 * needed for USB device implementations that are passed to the UsbDevice
 * template class.
 *
 * (Note that this is not a virtual interface.  Implementations do not need to
 * inherit from this class.)
 */
class UsbDeviceExample {
  /**
   * set_configuration() will be called in response to a SET_CONFIGURATION
   * control request.
   *
   * If the config_id is 0, the implementation should call
   * EndpointManager::unconfigure() to close all open endpoints and interfaces,
   * then return true.
   *
   * Otherwise if config_id refers to a valid configuration, the implementation
   * should configure the necessary interfaces and endpoints on the
   * EndpointManager, call EndpointManager::set_configured(), and then return
   * true.
   *
   * If the config_id does not refer to a valid configuration, false should be
   * returned.
   *
   * Note that set_configured() may be called to change the configuration ID
   * when the device is already configured.
   */
  bool set_configuration(uint8_t config_id, EndpointManager &ep_mgr) {
    return false;
  }

  /**
   * make_descriptor_map() will be called at compile time to create the
   * descriptor map.
   *
   * This should return a StaticDescriptorMap object, which will then be placed
   * into the firmware's read-only data segment.
   */
  static constexpr auto make_descriptor_map() {
    return StaticDescriptorMap();
  }
};

} // namespace ausb::device::example
