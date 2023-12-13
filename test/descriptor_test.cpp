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

#if 0
void dump_hex(const uint8_t* buf, uint16_t size) {
  auto p = buf;
  size_t bytes_left = size;
  while (bytes_left > 8) {
    printf("- %02x %02x %02x %02x %02x %02x %02x %02x\n",
           p[0],
           p[1],
           p[2],
           p[3],
           p[4],
           p[5],
           p[6],
           p[7]);
    p += 8;
    bytes_left -= 8;
  }
  if (bytes_left > 0) {
    printf("-");
    while (bytes_left > 0) {
      printf(" %02x", p[0]);
      ++p;
      --bytes_left;
    }
    printf("\n");
  }
}

void dump_desc(uint16_t value, uint16_t index) {
  printf("Descriptor %#x  %#x:\n", value, index);
  auto desc = usb.descriptor_map().get_descriptor_with_setup_ids(value, index);
  if (!desc.has_value()) {
    printf("- none\n");
    return;
  }

  printf("- size: %d\n", desc->size());
  dump_hex(desc->data(), desc->size());
}

void dump_descriptors() {
  printf("USB Descriptors:\n");
  dump_desc(0x100, 0);
  dump_desc(0x200, 0);
  dump_desc(0x300, 0);
  dump_desc(0x301, 0x0409);
  dump_desc(0x302, 0x0409);
  dump_desc(0x303, 0x0409);
  dump_desc(0x2200, 0);
}

[[nodiscard]] std::error_code run_test() {
  ESP_LOGI(LogTag, "Starting USB initialization...");
  const auto init_err = usb.init();
  if (init_err) {
      ESP_LOGE(LogTag, "Error initializing USB device.");
      return init_err;
  }
  ESP_LOGI(LogTag, "USB initialization complete.");

  size_t n = 0;
  while (true) {
    ++n;
    const auto event = usb.wait_for_event(10000ms);
    if (std::holds_alternative<NoEvent>(event)) {
      ESP_LOGI(LogTag, "usb %zu: no event", n);
    } else {
      ESP_LOGI(LogTag, "usb %zu: got event", n);
      usb.handle_event(event);
    }
  }

  return std::error_code();
}
#endif

const auto kDescriptors = make_descriptor_map();

} // namespace


ASEL_TEST(Descriptors, test) {
  auto dev_desc = kDescriptors.get_descriptor(DescriptorType::Device);
  ASEL_ASSERT_TRUE(dev_desc);
  ASEL_EXPECT_EQ(dev_desc->size(), 18);
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


} // namespace ausb
