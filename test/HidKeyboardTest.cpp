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
#include "ausb/hid/KeyboardInterface.h"
#include "ausb/hw/mock/MockDevice.h"
#include "ausb/log.h"
#include "test/lib/mock_utils.h"

#include <asel/test/checks.h>
#include <asel/test/TestCase.h>

using namespace ausb::device;

namespace ausb::test {

namespace {

class HidInEndpoint : public InEndpoint {
public:
  constexpr HidInEndpoint() = default;

  device::CtrlOutXfer *process_out_setup(device::MessagePipe *pipe,
                                         const SetupPacket &packet) override {
    // TODO
    return nullptr;
  }
  device::CtrlInXfer *process_in_setup(device::MessagePipe *pipe,
                                       const SetupPacket &packet) override {
    // TODO
    return nullptr;
  }
  void on_in_xfer_complete() override {
    // TODO
  }
  void on_in_xfer_failed(XferFailReason reason) override {
    // TODO
  }
};

constinit hid::KeyboardInterface kbd_intf;

// TODO: specify endpoint type and max packet size
constinit HidInEndpoint kbd_in_endpoint;

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
