// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/hid/usage/usage_page.h"

namespace ausb::hid {

/**
 * Usage Codes for the Generic Desktop Usage Page (0x01)
 *
 * Defined in the HID Usage Tables v1.3 spec:
 * https://usb.org/sites/default/files/hut1_3_0.pdf
 * (Section 4, page 32)
 */
enum class GenericDesktopUsage : uint8_t {
  Undefined = 0x00,
  Pointer = 0x01,
  Mouse = 0x02,
  Joystick = 0x04,
  Gamepad = 0x05,
  Keyboard = 0x06,
  Keypad = 0x07,
  MultiAxisController = 0x08,
  // More values are defined in the HID Usage Tables spec...
};
template <>
class is_usage_type<GenericDesktopUsage> : public std::true_type {};

} // namespace ausb::hid
