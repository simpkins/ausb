// Copyright (c) 2023, Adam Simpkins
#pragma once

#include <cstdint>

namespace ausb {

/**
 * HID class devices usually do not use the subclass field,
 * and instead use HID report descriptors to describe their data schema.
 *
 * However, HID report descriptors are complicated to parse, so HID supports
 * two pre-defined "boot" protocols for use by keyboards and mice.  This allows
 * BIOS code to work with HID keyboards and mice without needing to support
 * full HID report descriptor parsing.
 */
enum class HidSubclass : uint8_t {
    None = 0,
    Boot = 1,
};

enum class HidRequest : uint8_t {
  GetReport = 0x01,
  GetIdle = 0x02,
  GetProtocol = 0x03,
  SetReport = 0x09,
  SetIdle = 0x0a,
  SetProtocol = 0x0b,
};

/**
 * The HidProtocol field should be None unless the subclass is set to
 * HidSubclass::Boot.
 */
enum class HidProtocol : uint8_t {
    None = 0,
    Keyboard = 1,
    Mouse = 2,
};

/**
 * Most HID devices leave the country code set to 0.
 * The country code may be used to help identify the type of key caps used on a
 * keyboard.
 */
enum class HidCountry : uint8_t {
  NotSupported = 0,
  Arabic = 1,
  Belgian = 2,
  CanadianBilingual = 3,
  CanadianFrench = 4,
  CzechRepublic = 5,
  Danish = 6,
  Finnish = 7,
  French = 8,
  German = 9,
  Greek = 10,
  Hebrew = 11,
  Hungary = 12,
  International = 13,
  Italian = 14,
  Japan = 15,
  Korean = 16,
  LatinAmerica = 17,
  Netherlands = 18,
  Norwegian = 19,
  Persian = 20,
  Poland = 21,
  Portuguese = 22,
  Russia = 23,
  Slovakia = 24,
  Spanish = 25,
  Swedish = 26,
  SwissFrench = 27,
  SwissGerman = 28,
  Switzerland = 29,
  Taiwan = 30,
  TurkishQ = 31,
  UK = 32,
  US = 33,
  Yugoslavia = 34,
  TurkishF = 35,
};

} // namespace ausb
