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

constinit hid::KeyboardInterface kbd_intf;

struct KbdReport1 {
  static constexpr uint8_t report_id = 1;
  static constexpr uint16_t report_size = 8;
  static constexpr uint8_t queue_capacity = 2;
};
struct MouseReport2 {
  static constexpr uint8_t report_id = 2;
  static constexpr uint16_t report_size = 3;
  static constexpr uint8_t queue_capacity = 2;
};

// TODO: specify endpoint type and max packet size
constinit hid::HidInEndpoint<hid::ReportInfo<0, asel::array<uint8_t, 8>>>
    kbd_in_endpoint(1, 8);
constinit hid::HidInEndpoint<KbdReport1, MouseReport2> dual_in_endpoint(2, 8);

class KeyboardConfig {
public:
  std::array<Interface *, 1> interfaces = {{&kbd_intf}};
  std::array<InEndpoint *, 1> in_endpoints = {{&kbd_in_endpoint}};
  std::array<OutEndpoint *, 0> out_endpoints = {};
};
constexpr KeyboardConfig kbd_config;

class TestDevice {
public:
  static constexpr uint8_t kConfigId = 1;

  bool set_configuration(uint8_t config_id, EndpointManager& ep_mgr) {
    if (config_id == 0) {
      ep_mgr.unconfigure();
      return true;
    }
    if (config_id != kConfigId) {
      return false;
    }

    auto res = ep_mgr.open_in_endpoint(
        1, &kbd_in_endpoint, EndpointType::Interrupt, 8);
    if (!res) {
      AUSB_LOGE("error opening IN endpoint 1");
    }

    ep_mgr.set_configured(config_id, &kbd_intf);
    return true;
  }

  static constexpr auto make_descriptor_map() {
    DeviceDescriptor dev;
    dev.set_vendor(0x6666); // Prototype product vendor ID
    dev.set_product(0x1000);
    dev.set_device_release(1, 0);

    InterfaceDescriptor kbd_intf(UsbClass::Hid,
                                 static_cast<uint8_t>(HidSubclass::Boot),
                                 static_cast<uint8_t>(HidProtocol::Keyboard));

    EndpointDescriptor ep1;
    ep1.set_address(Direction::In, 1);
    ep1.set_type(EndpointType::Interrupt);
    ep1.set_interval(10);
    ep1.set_max_packet_size(8);

    auto kbd_report = hid::make_kbd_report_descriptor();

    HidDescriptor kbd_hid_desc;
    kbd_hid_desc.set_num_descriptors(1);
    kbd_hid_desc.set_report_descriptor_type(DescriptorType::HidReport);
    kbd_hid_desc.set_report_descriptor_length(kbd_report.kTotalLength);

    auto cfg = ConfigDescriptor(kConfigId, ConfigAttr::RemoteWakeup)
                   .add_interface(kbd_intf)
                   .add_descriptor(kbd_hid_desc)
                   .add_endpoint(ep1);

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
  hid::KeyboardInterface kbd_intf_;
};

constinit UsbDevice<TestDevice, MockDevice> usb;

} // namespace

ASEL_TEST(HidKeyboard, test) {
  attach_mock_device(usb);
}

} // namespace ausb::test
