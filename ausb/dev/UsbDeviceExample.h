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
   * control request with a non-zero config ID.
   *
   * The implementation should return false if this is an invalid config ID.
   * If the config ID is valid, it should configure the necessary interfaces
   * and endpoints on the EndpointManager, call
   * EndpointManager::set_configured(), and then return true.
   *
   * Note that set_configured() may be called to change the configuration ID
   * when the device is already configured, without an intervening
   * unconfigure() call.
   */
  bool set_configuration(uint8_t config_id, EndpointManager *ep_mgr) {
    return false;
  }

  /**
   * unconfigure() will be called in response to a SET_CONFIGURATION control
   * request with a config ID set to 0.
   *
   * The implementation should call EndpointManager::unconfigure() to close all
   * endpoints and put the device back into the "Address" state.
   */
  void unconfigure(EndpointManager *ep_mgr) { ep_mgr->unconfigure(); }

  /**
   * make_descriptor_map() will be called at compile time to create the
   * descriptor map.
   *
   * This should return a StaticDescriptorMap object, which will then be placed
   * into the firmware's read-only data segment.
   */
  static constexpr auto make_descriptor_map() { return StaticDescriptorMap(); }
};

} // namespace ausb::device::example
