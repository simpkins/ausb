// Copyright (c) 2023, Adam Simpkins
#pragma once

/*
 * Type definitions for use in HID report descriptors.
 */

#include <cstdint>

namespace ausb::hid {

// ItemPrefix defines the upper 6 bits for the item prefix byte.
// The lower two bits are used for size bits.
enum class ItemPrefix {
  // Main items
  // These identify data fields in the report
  Input = 0x80,
  Output = 0x90,
  Feature = 0xb0,
  Collection = 0xa0,
  EndCollection = 0xc0,
  // Global items
  // These set properties for all subsequent Main items in the
  // report descriptor
  UsagePage = 0x04,
  LogicalMinimum = 0x14,
  LogicalMaximum = 0x24,
  PhysicalMinimum = 0x34,
  PhysicalMaximum = 0x44,
  UnitExponent = 0x54,
  Unit = 0x64,
  ReportSize = 0x74,
  ReportID = 0x84,
  ReportCount = 0x94,
  Push = 0xa4,
  Pop = 0xb4,
  // Local items
  // These set properties for the next Main item in the report descriptor, but
  // do not apply to other following Main items after that.
  Usage = 0x08,
  UsageMinimum = 0x18,
  UsageMaximum = 0x28,
  DesignatorIndex = 0x38,
  DesignatorMinimum = 0x48,
  DesignatorMaximum = 0x58,
  StringIndex = 0x78,
  StringMinimum = 0x88,
  StringMaximum = 0x98,
  Delimiter = 0xa8,
  // This prefix indicates that this item is encoded using the long item
  // format
  LongFormat = 0xfe,
};

enum class CollectionType {
  Physical = 0x00,    // e.g., group of axes
  Application = 0x01, // e.g., mouse, keyboard
  Logical = 0x02,     // interrelated data
  Report = 0x03,
  NamedArray = 0x04,
  UsageSwitch = 0x05,
  UsageModifier = 0x06,
  // Values 0x07 - 0x7f are reserved
  // Values 0x80 - 0xff are for vendor-defined values
};

template <typename Self, typename T>
class MainTypeBase {
public:
  using data_type = T;

  constexpr MainTypeBase() noexcept = default;
  constexpr explicit MainTypeBase(T value) noexcept : value_(value) {}

  constexpr Self &data() {
    return clear_bit(0);
  }
  constexpr Self &constant() {
    return set_bit(0);
  }

  constexpr Self &array() {
    return clear_bit(1);
  }
  constexpr Self &variable() {
    return set_bit(1);
  }

  constexpr Self &absolute() {
    return clear_bit(2);
  }
  constexpr Self &relative() {
    return set_bit(2);
  }

  constexpr Self &no_wrap() {
    return clear_bit(3);
  }
  constexpr Self &wrap() {
    return set_bit(3);
  }

  constexpr Self &linear() {
    return clear_bit(4);
  }
  constexpr Self &non_linear() {
    return set_bit(4);
  }

  constexpr Self &preferred_state() {
    return clear_bit(5);
  }
  constexpr Self &no_preferred_state() {
    return set_bit(5);
  }

  constexpr Self &no_null_position() {
    return clear_bit(6);
  }
  constexpr Self &null_position() {
    return set_bit(6);
  }

protected:
  constexpr Self &set_bit(uint8_t n) {
    value_ |= (1 << n);
    return *static_cast<Self *>(this);
  }
  constexpr Self &clear_bit(uint8_t n) {
    value_ &= ~(1 << n);
    return *static_cast<Self *>(this);
  }

  T value_ = 0;
};

class Input : public MainTypeBase<Input, uint8_t> {
public:
  using MainTypeBase::MainTypeBase;

  constexpr uint8_t u8() const {
    return value_;
  }
};

class InputU16 : public MainTypeBase<InputU16, uint16_t> {
public:
  using MainTypeBase::MainTypeBase;

  constexpr uint8_t u16() const {
    return value_;
  }

  constexpr InputU16 &bit_field() {
    return clear_bit(8);
  }
  constexpr InputU16 &buffered_bytes() {
    return set_bit(8);
  }
};

class Output : public MainTypeBase<Output, uint8_t> {
public:
  using MainTypeBase::MainTypeBase;

  constexpr Output &non_volatile() {
    return clear_bit(7);
  }
  constexpr Output &is_volatile() {
    return set_bit(7);
  }

  constexpr uint8_t u8() const {
    return value_;
  }
};

class OutputU16 : public MainTypeBase<OutputU16, uint16_t> {
public:
  using MainTypeBase::MainTypeBase;

  constexpr uint8_t u16() const {
    return value_;
  }

  constexpr OutputU16 &non_volatile() {
    return clear_bit(7);
  }
  constexpr OutputU16 &is_volatile() {
    return set_bit(7);
  }

  constexpr OutputU16 &bit_field() {
    return clear_bit(8);
  }
  constexpr OutputU16 &buffered_bytes() {
    return set_bit(8);
  }
};

class Feature : public MainTypeBase<Feature, uint8_t> {
public:
  using MainTypeBase::MainTypeBase;

  constexpr Feature &non_volatile() {
    return clear_bit(7);
  }
  constexpr Feature &is_volatile() {
    return set_bit(7);
  }

  constexpr uint8_t u8() const {
    return value_;
  }
};

class FeatureU16 : public MainTypeBase<FeatureU16, uint16_t> {
public:
  using MainTypeBase::MainTypeBase;

  constexpr uint8_t u16() const {
    return value_;
  }

  constexpr FeatureU16 &non_volatile() {
    return clear_bit(7);
  }
  constexpr FeatureU16 &is_volatile() {
    return set_bit(7);
  }

  constexpr FeatureU16 &bit_field() {
    return clear_bit(8);
  }
  constexpr FeatureU16 &buffered_bytes() {
    return set_bit(8);
  }
};

} // namespace ausb::hid
