// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/hid/HidInterface.h"
#include "ausb/hid/HidReportDescriptor.h"
#include "ausb/hid/generic_desktop.h"
#include "ausb/hid/key_codes.h"
#include "ausb/hid/leds.h"
#include "ausb/hid/usage_page.h"

namespace ausb::hid {

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

class KeyboardInterface : public HidInterface {
public:
  constexpr KeyboardInterface() noexcept = default;

  std::unique_ptr<device::CtrlOutXfer>
  process_out_setup(device::ControlEndpoint *ctrl_ep,
                    const SetupPacket &packet) override;
  std::unique_ptr<device::CtrlInXfer>
  process_in_setup(device::ControlEndpoint *ctrl_ep,
                   const SetupPacket &packet) override;

  bool set_report(asel::buf_view data) override;

  static constexpr size_t report_descriptor_length() {
    return report_descriptor_.kTotalLength;
  }

private:
  static constexpr auto report_descriptor_ = make_kbd_report_descriptor();
};

} // namespace ausb::hid