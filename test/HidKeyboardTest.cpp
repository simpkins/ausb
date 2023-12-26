// Copyright (c) 2023, Adam Simpkins

#include "ausb/UsbDevice.h"
#include "ausb/desc/ConfigDescriptor.h"
#include "ausb/desc/DeviceDescriptor.h"
#include "ausb/desc/EndpointDescriptor.h"
#include "ausb/desc/StaticDescriptorMap.h"
#include "ausb/dev/EndpointManager.h"
#include "ausb/dev/InEndpoint.h"
#include "ausb/dev/OutEndpoint.h"
#include "ausb/hid/HidDescriptor.h"
#include "ausb/hid/HidInEndpoint.h"
#include "ausb/hid/KeyboardInterface.h"
#include "ausb/hw/mock/MockDevice.h"
#include "ausb/log.h"
#include "test/lib/mock_utils.h"

#include <asel/test/checks.h>
#include <asel/test/TestCase.h>

using namespace ausb::device;

namespace ausb::test {

namespace {

class TestDevice {
public:
  static constexpr uint8_t kConfigId = 1;
  static constexpr uint8_t kHidInEndpointNum = 1;

  bool set_configuration(uint8_t config_id, EndpointManager& ep_mgr) {
    if (config_id == 0) {
      ep_mgr.unconfigure();
      return true;
    }
    if (config_id != kConfigId) {
      return false;
    }

    auto res =
        ep_mgr.open_in_endpoint(kHidInEndpointNum,
                                &kbd_intf_.in_endpoint(),
                                EndpointType::Interrupt,
                                hid::KeyboardInterface::kDefaultMaxPacketSize);
    if (!res) {
      AUSB_LOGE("error opening HID IN endpoint");
    }

    ep_mgr.set_configured(config_id, &kbd_intf_);
    return true;
  }

  static constexpr auto make_descriptor_map() {
    DeviceDescriptor dev;
    dev.set_vendor(0x6666); // Prototype product vendor ID
    dev.set_product(0x1000);
    dev.set_device_release(1, 0);

    auto cfg = hid::KeyboardInterface::update_config_descriptor(
        ConfigDescriptor(kConfigId, ConfigAttr::RemoteWakeup),
        kHidInEndpointNum);

    return StaticDescriptorMap()
        .add_device_descriptor(dev)
        .add_language_ids(Language::English_US)
        .add_string(dev.mfgr_str_idx(), "ACME, Inc.", Language::English_US)
        .add_string(dev.product_str_idx(), "AUSB Test Device",
                    Language::English_US)
        .add_string(dev.serial_str_idx(), "00:00:00::00:00:00",
                    Language::English_US)
        .add_config_descriptor(cfg);
  }

private:
  hid::KeyboardInterface kbd_intf_{kHidInEndpointNum};
};

constinit UsbDevice<TestDevice, MockDevice> usb;

} // namespace

ASEL_TEST(HidKeyboard, test) {
  attach_mock_device(usb);
}

} // namespace ausb::test
