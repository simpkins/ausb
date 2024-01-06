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
#include "ausb/hid/kbd/BootKeyboard.h"
#include "ausb/hw/mock/MockDevice.h"
#include "ausb/log.h"
#include "test/lib/mock_utils.h"

#include <asel/test/checks.h>
#include <asel/test/TestCase.h>

using namespace ausb::device;
using ausb::kbd::BootKeyboard;

namespace ausb::test {

namespace {

class TestDevice {
public:
  static constexpr uint8_t kConfigId = 1;
  static constexpr uint8_t kHidInEndpointNum = 1;

  constexpr explicit TestDevice(EndpointManager *manager)
      : kbd_intf_(manager, kHidInEndpointNum) {}

  BootKeyboard &kbd_intf() {
    return kbd_intf_;
  }

  bool set_configuration(uint8_t config_id, EndpointManager& ep_mgr) {
    if (config_id == 0) {
      ep_mgr.unconfigure();
      return true;
    }
    if (config_id != kConfigId) {
      return false;
    }

    auto res = ep_mgr.open_in_endpoint(kHidInEndpointNum,
                                       &kbd_intf_.in_endpoint(),
                                       EndpointType::Interrupt,
                                       BootKeyboard::kDefaultMaxPacketSize);
    if (!res) {
      AUSB_LOGE("error opening HID IN endpoint");
      return false;
    }

    ep_mgr.set_configured(config_id, &kbd_intf_);
    return true;
  }

  static constexpr auto make_descriptor_map() {
    DeviceDescriptor dev;
    dev.set_vendor(0x6666); // Prototype product vendor ID
    dev.set_product(0x1000);
    dev.set_device_release(1, 0);

    auto cfg = BootKeyboard::update_config_descriptor(
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
  BootKeyboard kbd_intf_;
};

constinit UsbDevice<TestDevice, MockDevice> usb;

} // namespace

ASEL_TEST(HidKeyboard, test) {
  attach_mock_device(usb);

  // Send a report
  usb.dev().kbd_intf().send_report(
      {0x71, 0, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09});
  ASEL_EXPECT_TRUE(usb.hw()->in_eps[1].xfer_in_progress);
  const auto expected =
      BootKeyboard::ReportType{{0x71, 0, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09}};
  ASEL_EXPECT_EQ(expected, usb.hw()->in_eps[1].cur_xfer_buf());

  // Try to send another report while the first report is still being
  // transmitted.  This one will be stored as the current state, but we can't
  // start transmitting it until the current transmission is complete.
  usb.dev().kbd_intf().send_report(
      {0x70, 0, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a});
  // Try to send another report while the first report is still being
  // transmitted.  This one will be stored as the current state, but we can't
  // start transmitting it until the current transmission is complete.
  // The previous untransmitted state will be dropped.
  usb.dev().kbd_intf().send_report(
      {0x80, 0, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b});

  usb.hw()->complete_in_xfer(1);
  ASEL_EXPECT_TRUE(usb.hw()->in_eps[1].xfer_in_progress);
  const auto expected2 =
      BootKeyboard::ReportType{{0x80, 0, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b}};
  ASEL_EXPECT_EQ(expected2, usb.hw()->in_eps[1].cur_xfer_buf());

  usb.hw()->complete_in_xfer(1);
  ASEL_EXPECT_FALSE(usb.hw()->in_eps[1].xfer_in_progress);
}

} // namespace ausb::test
