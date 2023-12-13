// Copyright (c) 2023, Adam Simpkins
#include "ausb/desc/ConfigDescriptor.h"
#include "ausb/desc/DeviceDescriptor.h"
#include "ausb/desc/EndpointDescriptor.h"
#include "ausb/desc/StaticDescriptorMap.h"
#include "ausb/hid/HidDescriptor.h"
#include "ausb/hid/HidReportDescriptor.h"
#include "ausb/hid/KeyboardInterface.h"
#include "ausb/hid/key_codes.h"
#include "ausb/hid/leds.h"
#include "ausb/hid/types.h"

#include <asel/test/checks.h>
#include <asel/test/TestCase.h>

namespace ausb {
namespace {

constexpr auto make_descriptor_map() {
  DeviceDescriptor dev;
  dev.set_vendor(0xcafe);
  dev.set_product(0xd00d);
  dev.set_class(UsbClass::Hid);
  dev.set_subclass(1);
  dev.set_protocol(2);
  dev.set_device_release(12, 34);

  InterfaceDescriptor kbd_intf(UsbClass::Hid,
                               static_cast<uint8_t>(HidSubclass::Boot),
                               static_cast<uint8_t>(HidProtocol::Keyboard));

  EndpointDescriptor ep1;
  ep1.set_address(Direction::In, 1);
  ep1.set_type(EndpointType::Interrupt);
  ep1.set_interval(10);
  ep1.set_max_packet_size(8);

  // TODO: move this definition to the ausb/hid code
  const auto max_key_code = hid::Key::ExSel;
  auto kbd_report = hid::ReportDescriptor()
                        .usage_page(hid::UsagePage::GenericDesktop)
                        .usage(hid::GenericDesktopUsage::Keyboard)
                        .collection(hid::CollectionType::Application)
                        // Modifier flags (input)
                        // 8 bits, each one representing a key code
                        // starting from LeftControl to RightGui
                        .report_size(1)
                        .report_count(8)
                        .usage_page(hid::UsagePage::KeyCodes)
                        .usage_min(hid::Key::LeftControl)
                        .usage_max(hid::Key::RightGui)
                        .logical_min(0)
                        .logical_max(1)
                        .input(hid::Input().data().variable())
                        // Reserved input byte
                        .report_count(1)
                        .report_size(8)
                        .input(hid::Input().constant().variable())
                        // LEDs (output, from host to device)
                        // 5 bits, each one representing an LED code
                        // starting from NumLock to Kana
                        .report_count(5)
                        .report_size(1)
                        .usage_page(hid::UsagePage::LEDs)
                        .usage_min(hid::Led::NumLock)
                        .usage_max(hid::Led::Kana)
                        .output(hid::Output().data().variable())
                        // 3 padding bits after the LED bits
                        .report_count(1)
                        .report_size(3)
                        .output(hid::Output().constant().variable())
                        // Key codes
                        // 6 bytes, each representing a key code.
                        .report_count(6)
                        .report_size(8)
                        .logical_min(0)
                        .logical_max_u8(static_cast<uint8_t>(max_key_code))
                        .usage_page(hid::UsagePage::KeyCodes)
                        .usage_min(hid::Key::None)
                        .usage_max(max_key_code)
                        .input(hid::Input().data().array())
                        .end_collection();

  HidDescriptor kbd_hid_desc;
  kbd_hid_desc.set_num_descriptors(1);
  kbd_hid_desc.set_report_descriptor_type(DescriptorType::HidReport);
  kbd_hid_desc.set_report_descriptor_length(kbd_report.kTotalLength);

  auto cfg = ConfigDescriptor(1, ConfigAttr::RemoteWakeup)
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

const auto kDescriptors = make_descriptor_map();

} // namespace

ASEL_TEST(Descriptors, device_descriptor) {
  auto dev_desc = kDescriptors.get_descriptor(DescriptorType::Device);
  ASEL_ASSERT_TRUE(dev_desc);
  std::array<uint8_t, 18> expected_dev_desc = {{
      18,   // length
      1,    // descriptor type: device
      0,    // USB minor version
      2,    // USB major version
      3,    // class
      1,    // subclass
      2,    // protocol
      64,   // max_packet size
      0xfe, // vendor (lower half)
      0xca, // vendor (upper half)
      0x0d, // product (lower half)
      0xd0, // product (upper half)
      0x34, // device version (lower half)
      0x12, // device version (upper half)
      1,    // manufacturer string index
      2,    // product string index
      3,    // serial string index
      1,    // num configurations
  }};
  ASEL_EXPECT_EQ(*dev_desc, expected_dev_desc);
}

ASEL_TEST(Descriptors, config_descriptor) {
  auto cfg_desc = kDescriptors.get_descriptor(DescriptorType::Config);
  ASEL_ASSERT_TRUE(cfg_desc);
  std::array<uint8_t, 34> expected_desc = {{
      9,    // config descriptor length
      2,    // descriptor type: config
      34,   // total length (lower half)
      0,    // total length (upper half)
      1,    // num interfaces
      1,    // value
      0,    // config name string index
      0x20, // attributes: remote wakeup suppported
      25,   // max power in 2ma
            //
      9,    // interface descriptor length
      4,    // descriptor type: interface
      0,    // interface number
      0,    // alt setting
      1,    // num endpoints
      3,    // class
      1,    // subclass
      1,    // protocol
      0,    // interface name string index
            //
      9,    // hid descriptor length
      0x21, // descriptor type: hid
      0x11, // HID minor version
      0x01, // HID major version
      0,    // country code
      1,    // num descriptors
      0x22, // descriptor type
      63,   // descriptor length (lower half)
      0,    // descriptor length (upper half)
            //
      7,    // endpoint descriptor length
      5,    // descriptor type: endpoint
      0x81, // endpoint address
      0x03, // attributes: interrupt type
      8,    // max packet size (lower half)
      0,    // max packet size (upper half)
      10,   // interval
  }};
  ASEL_EXPECT_EQ(*cfg_desc, expected_desc);
}

} // namespace ausb
