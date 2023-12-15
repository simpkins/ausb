// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/hid/usage_page.h"

namespace ausb::hid {

/**
 * Usage Codes for the LED Usage Page (0x08)
 *
 * Defined in the HID Usage Tables v1.3 spec:
 * https://usb.org/sites/default/files/hut1_3_0.pdf
 * (Section 11, page 96)
 */
enum class Led : uint8_t {
  None = 0x00,
  NumLock = 0x01,
  CapsLock = 0x02,
  ScrollLock = 0x03,
  Compose = 0x04,
  Kana = 0x05,
  Power = 0x06,
  Shift = 0x07,
  DoNotDisturb = 0x08,
  Mute = 0x09,
  ToneEnable = 0x0a,
  HighCutFilter = 0x0b,
  LowCutFilter = 0x0c,
  EqualizerEnable = 0x0d,
  SoundFieldOn = 0x0e,
  SurroundOn = 0x0f,
  // More values are defined in the HID Usage Tables spec...
};
template <>
class is_usage_type<Led> : public std::true_type {};

} // namespace ausb::hid
