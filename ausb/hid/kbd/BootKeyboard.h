// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/desc/ConfigDescriptor.h"
#include "ausb/desc/EndpointDescriptor.h"
#include "ausb/hid/HidDescriptor.h"
#include "ausb/hid/HidInEndpoint.h"
#include "ausb/hid/HidInterface.h"
#include "ausb/hid/HidReportDescriptor.h"
#include "ausb/hid/HidReportQueue.h"
#include "ausb/hid/usage/generic_desktop.h"
#include "ausb/hid/usage/keys.h"
#include "ausb/hid/usage/leds.h"
#include "ausb/hid/usage/usage_page.h"

namespace ausb::kbd {

static constexpr auto make_kbd_report_descriptor() {
  const auto max_key_code = hid::Key::ExSel;
  return hid::ReportDescriptor()
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
}

class KeyboardLedCallback {
public:
  virtual void on_set_leds(uint8_t leds) = 0;
};

class BootKeyboard : public hid::HidInterface,
                     private hid::HidInterfaceCallback {
public:
  static constexpr uint16_t kDefaultMaxPacketSize = 8;
  static constexpr uint8_t kDefaultInterval = 10;
  static constexpr uint8_t kReportId = 0;
  using ReportType = std::array<uint8_t, 8>;
  using KbdReportInfo = hid::ReportInfo<0, ReportType>;

  constexpr explicit BootKeyboard(
      device::EndpointManager *manager,
      uint8_t in_endpoint_num,
      KeyboardLedCallback *callback = nullptr,
      uint16_t max_packet_size = kDefaultMaxPacketSize) noexcept
      : HidInterface(manager,
                     in_endpoint_num,
                     max_packet_size,
                     report_descriptor_.data(),
                     &report_map_,
                     &report_map_,
                     this),
        callback_(callback) {}

  void send_report(const uint8_t *data);
  void send_report(const ReportType &data) {
    send_report(data.data());
  }

  // TODO: add a thread-safe method to set the report from a different task

  static constexpr InterfaceDescriptor make_interface_descriptor() {
    return HidInterface::make_boot_interface_descriptor(HidProtocol::Keyboard);
  }

  static constexpr EndpointDescriptor
  make_in_endpoint_descriptor(uint8_t endpoint_num,
                              uint16_t max_packet_size = kDefaultMaxPacketSize,
                              uint8_t interval = kDefaultInterval) {
    EndpointDescriptor desc;
    desc.set_address(Direction::In, endpoint_num);
    desc.set_type(EndpointType::Interrupt);
    desc.set_interval(interval);
    desc.set_max_packet_size(max_packet_size);
    return desc;
  }

  template <size_t TotalLength, uint8_t NumInterfaces>
  static constexpr auto
  update_config_descriptor(ConfigDescriptor<TotalLength, NumInterfaces> cfg,
                           uint8_t endpoint_num,
                           uint16_t max_packet_size = 8,
                           uint8_t interval = 10) {
    auto report_desc = make_kbd_report_descriptor();

    HidDescriptor hid_desc;
    hid_desc.set_num_descriptors(1);
    hid_desc.set_report_descriptor_type(DescriptorType::HidReport);
    hid_desc.set_report_descriptor_length(report_desc.kTotalLength);

    return cfg.add_interface(make_interface_descriptor())
        .add_descriptor(hid_desc)
        .add_endpoint(make_in_endpoint_descriptor(
            endpoint_num, max_packet_size, interval));
  }

private:
  class SetLedHandler;

  device::CtrlOutXfer *set_report(device::MessagePipe *pipe,
                                  const SetupPacket &packet,
                                  HidReportType report_type,
                                  uint8_t report_id) override;
  void set_leds(uint8_t value);

  static constexpr auto report_descriptor_ = make_kbd_report_descriptor();

  hid::HidReportMapStorage<KbdReportInfo> report_map_;
  KeyboardLedCallback *callback_ = nullptr;
};

} // namespace ausb::kbd
