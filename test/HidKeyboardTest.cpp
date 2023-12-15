// Copyright (c) 2023, Adam Simpkins

#include "ausb/UsbDevice.h"
#include "ausb/desc/ConfigDescriptor.h"
#include "ausb/desc/DeviceDescriptor.h"
#include "ausb/desc/EndpointDescriptor.h"
#include "ausb/desc/StaticDescriptorMap.h"
#include "ausb/dev/EndpointManager.h"
#include "ausb/hid/HidDescriptor.h"
#include "ausb/hid/KeyboardInterface.h"
#include "ausb/hw/mock/MockDevice.h"

#include <asel/test/checks.h>
#include <asel/test/TestCase.h>

using namespace ausb::device;

namespace ausb::test {

namespace {

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

    ep_mgr.add_interface(0, &kbd_intf_);
#if 0
    // TODO:
    // ep_mgr.open_in_endpoint(1);
    auto res = ep_mgr.hw()->open_in_endpoint(1, EndpointType::Interrupt, 8);
    if (!res) {
      AUSB_LOGE("error opening IN endpoint 1");
    }
#endif

    ep_mgr.set_configured();
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
        .add_string(dev.mfgr_str_idx(), "Adam Simpkins", Language::English_US)
        .add_string(dev.product_str_idx(), "AUSB Test Device",
                    Language::English_US)
        .add_string(dev.serial_str_idx(), "00:00:00::00:00:00",
                    Language::English_US)
        .add_config_descriptor(cfg);
  }

private:
  hid::KeyboardInterface kbd_intf_;
};

static constinit UsbDevice<TestDevice, MockDevice> usb;

bool init_device() {
  usb.hw()->reset();
  const auto init_err = usb.init();
  if (!ASEL_EXPECT_FALSE(init_err)) {
    return false;
  }

  usb.handle_event(SuspendEvent{});
  usb.handle_event(BusResetEvent{});
  usb.handle_event(BusEnumDone{UsbSpeed::Full});

  // Walk through some normal steps that would be done by a host
  // Get the device descriptor
  SetupPacket get_dev_desc;
  get_dev_desc.request_type = 0x80; // IN, Device, Standard request
  get_dev_desc.request = 6;         // GET_DESCRIPTOR
  get_dev_desc.value = 0x0100;      // Device descriptor
  get_dev_desc.index = 0;
  get_dev_desc.length = 18;
  usb.handle_event(SetupPacketEvent{0, get_dev_desc});
  if (!ASEL_EXPECT_TRUE(usb.hw()->in_eps[0].xfer_in_progress)) {
    return false;
  }
  buf_view dd_reply(
      static_cast<const uint8_t *>(usb.hw()->in_eps[0].cur_xfer_data),
      usb.hw()->in_eps[0].cur_xfer_size);
  if (!ASEL_EXPECT_EQ(dd_reply.size(), DeviceDescriptorParser::kSize)) {
    return false;
  }

  DeviceDescriptorParser dd(dd_reply);
  const auto num_configs = dd.num_configs();
  if (!ASEL_EXPECT_GE(num_configs, 1)) {
      return false;
  }

  usb.handle_event(usb.hw()->complete_in_xfer(0));
  if (!ASEL_EXPECT_TRUE(usb.hw()->out_eps[0].xfer_in_progress)) {
    return false;
  }
  ASEL_EXPECT_EQ(0, usb.hw()->out_eps[0].cur_xfer_size);
  usb.handle_event(usb.hw()->complete_out_xfer(0));

  // Call SET_ADDRESS
  SetupPacket set_addr;
  set_addr.request_type = 0x00; // Out, Device, Standard request
  set_addr.request = 5;         // SET_ADDRESS
  set_addr.value = 1234;        // Address
  set_addr.index = 0;
  set_addr.length = 0;
  usb.handle_event(SetupPacketEvent{0, set_addr});
  if (!ASEL_EXPECT_TRUE(usb.hw()->in_eps[0].xfer_in_progress)) {
    return false;
  }
  ASEL_EXPECT_EQ(0, usb.hw()->in_eps[0].cur_xfer_size);
  usb.handle_event(usb.hw()->complete_in_xfer(0));

  // Get the config descriptor
  SetupPacket get_cfg_desc;
  get_cfg_desc.request_type = 0x80; // IN, Device, Standard request
  get_cfg_desc.request = 6;         // GET_DESCRIPTOR
  get_cfg_desc.value = 0x0200;      // Config descriptor
  get_cfg_desc.index = 0;
  get_cfg_desc.length = 9;
  usb.handle_event(SetupPacketEvent{0, get_cfg_desc});
  if (!ASEL_EXPECT_TRUE(usb.hw()->in_eps[0].xfer_in_progress)) {
    return false;
  }
  buf_view cfg_reply(
      static_cast<const uint8_t *>(usb.hw()->in_eps[0].cur_xfer_data),
      usb.hw()->in_eps[0].cur_xfer_size);
  if (!ASEL_EXPECT_EQ(cfg_reply.size(), ConfigDescriptorParser::kSize)) {
    return false;
  }

  ConfigDescriptorParser cfg_desc(cfg_reply);
  const auto cfg_id = cfg_desc.value();

  usb.handle_event(usb.hw()->complete_in_xfer(0));
  if (!ASEL_EXPECT_TRUE(usb.hw()->out_eps[0].xfer_in_progress)) {
    return false;
  }
  ASEL_EXPECT_EQ(0, usb.hw()->out_eps[0].cur_xfer_size);
  usb.handle_event(usb.hw()->complete_out_xfer(0));

  // Call SET_CONFIGURATION with the first config ID
  SetupPacket set_cfg_desc;
  set_cfg_desc.request_type = 0x00; // Out, Device, Standard request
  set_cfg_desc.request = 9;         // SET_CONFIGURATION
  set_cfg_desc.value = cfg_id;      // Config ID
  set_cfg_desc.index = 0;
  set_cfg_desc.length = 0;
  usb.handle_event(SetupPacketEvent{0, set_cfg_desc});
  if (!ASEL_EXPECT_TRUE(usb.hw()->in_eps[0].xfer_in_progress)) {
    return false;
  }
  ASEL_EXPECT_EQ(0, usb.hw()->in_eps[0].cur_xfer_size);
  usb.handle_event(usb.hw()->complete_in_xfer(0));

  return true;
}

} // namespace

ASEL_TEST(HidKeyboard, test) {
  init_device();

}

} // namespace ausb::test
