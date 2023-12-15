// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/UsbDevice.h"

namespace ausb {
class MockDevice;
}

namespace ausb::test {

/**
 * Drive a MockDevice through steps as if it were attached to a bus and
 * configured by a host.
 *
 * This is mainly intended as a convenience functions that unit tests can use
 * to get the device into a normal state so they can begin functionality
 * testing.
 *
 * This walks through the steps that a host would do to set the address and the
 * configuration.  If anything goes wrong during this process
 * ASEL_ADD_FAILURE() is called to mark the current test as a failure, and
 * false is returned.  True is returned on success.
 */
bool attach_mock_device(MockDevice *hw, device::EndpointManager *ep_manager);
template <typename UsbDeviceImpl, typename HwDeviceType>
bool attach_mock_device(device::UsbDevice<UsbDeviceImpl, HwDeviceType> &usb) {
  return attach_mock_device(usb.hw(), usb.manager());
}

} // namespace ausb::test
