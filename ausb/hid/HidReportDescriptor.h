// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/hid/report_types.h"
#include "ausb/hid/usage/usage_page.h"

#include <array>
#include <type_traits>

namespace ausb::hid {

/**
 * A class for constructing HID Report Descriptors.
 *
 * This class is primarily intended to be used for constructing descriptors at
 * compile time, so they can be embedded in an binary's read-only data section.
 *
 * Note that all of the methods in ReportDescriptor for adding new items
 * return a new ReportDescriptor object with the additional item appended,
 * rather than modifying the current object in place.  Unlike many other USB
 * descriptor types, a HID Report Descriptor is an array of items rather than a
 * table with fixed structure.  Any time we want to append an item we must
 * return a new descriptor object with additional space for the new item.
 */
template <size_t TotalLength = 0>
class ReportDescriptor {
public:
  constexpr ReportDescriptor() requires(TotalLength == 0) = default;

  static constexpr size_t kTotalLength = TotalLength;

  constexpr const std::array<uint8_t, kTotalLength> &data() const {
    return data_;
  }
  constexpr std::array<uint8_t, kTotalLength> &data() {
    return data_;
  }

  /************
   * Main Items
   ************/

  /**
   * Return a new descriptor with an Input item appended.
   */
  constexpr ReportDescriptor<TotalLength + 2> input(Input input) {
    return input_u8(input.u8());
  }
  constexpr ReportDescriptor<TotalLength + 3> input(InputU16 input) {
    return input_u16(input.u16());
  }
  constexpr ReportDescriptor<TotalLength + 2> input_u8(uint8_t input) {
    return add_short_item_u8(ItemPrefix::Input, input);
  }
  constexpr ReportDescriptor<TotalLength + 3> input_u16(uint16_t input) {
    return add_short_item_u16(ItemPrefix::Input, input);
  }

  /**
   * Return a new descriptor with an Output item appended.
   */
  constexpr ReportDescriptor<TotalLength + 2> output(Output output) {
    return output_u8(output.u8());
  }
  constexpr ReportDescriptor<TotalLength + 3> output(OutputU16 output) {
    return output_u16(output.u16());
  }
  constexpr ReportDescriptor<TotalLength + 2> output_u8(uint8_t output) {
    return add_short_item_u8(ItemPrefix::Output, output);
  }
  constexpr ReportDescriptor<TotalLength + 3> output_u16(uint16_t output) {
    return add_short_item_u16(ItemPrefix::Output, output);
  }

  /**
   * Return a new descriptor with a Feature item appended.
   */
  constexpr ReportDescriptor<TotalLength + 2> feature(Feature feature) {
    return feature_u8(feature.u8());
  }
  constexpr ReportDescriptor<TotalLength + 3> feature(FeatureU16 feature) {
    return feature_u16(feature.u16());
  }
  constexpr ReportDescriptor<TotalLength + 2> feature_u8(uint8_t feature) {
    return add_short_item_u8(ItemPrefix::Feature, feature);
  }
  constexpr ReportDescriptor<TotalLength + 3> feature_u16(uint16_t feature) {
    return add_short_item_u16(ItemPrefix::Feature, feature);
  }

  /**
   * Return a new descriptor with a Collection item appended.
   *
   * end_collection() should be called later to end the collection.
   */
  constexpr ReportDescriptor<TotalLength + 2> collection(CollectionType type) {
    return collection(static_cast<uint8_t>(type));
  }
  constexpr ReportDescriptor<TotalLength + 2> collection(uint8_t type) {
    return add_short_item_u8(ItemPrefix::Collection, type);
  }

  /**
   * Return a new descriptor with an EndCollection item appended.
   */
  constexpr ReportDescriptor<TotalLength + 1> end_collection() {
    return add_short_item(ItemPrefix::EndCollection);
  }

  /**************
   * Global Items
   **************/

  /**
   * Return a new descriptor with a UsagePage item appended.
   */
  constexpr ReportDescriptor<TotalLength + 2> usage_page(hid::UsagePage value) {
    return add_short_item_u8(ItemPrefix::UsagePage,
                             static_cast<uint8_t>(value));
  }
  constexpr ReportDescriptor<TotalLength + 2> usage_page_u8(uint8_t value) {
    return add_short_item_u8(ItemPrefix::UsagePage, value);
  }
  constexpr ReportDescriptor<TotalLength + 3> usage_page_u16(uint16_t value) {
    return add_short_item_u16(ItemPrefix::UsagePage, value);
  }

  /**
   * Return a new descriptor with a LogicalMinimum item appended.
   */
  constexpr ReportDescriptor<TotalLength + 2> logical_min(int8_t value) {
    return add_short_item_i8(ItemPrefix::LogicalMinimum, value);
  }
  constexpr ReportDescriptor<TotalLength + 2> logical_min_i16(int16_t value) {
    return add_short_item_i16(ItemPrefix::LogicalMinimum, value);
  }
  constexpr ReportDescriptor<TotalLength + 2> logical_min_i32(int32_t value) {
    return add_short_item_i32(ItemPrefix::LogicalMinimum, value);
  }

  /**
   * Return a new descriptor with a LogicalMaximum item appended.
   *
   * Note that LogicalMaximum is treated as signed if the current
   * LogicalMinimum state is negative, and unsigned otherwise.  (At least
   * according to the report parsing logic in the Linux kernel).
   */
  constexpr ReportDescriptor<TotalLength + 2> logical_max(int8_t value) {
    return add_short_item_i8(ItemPrefix::LogicalMaximum, value);
  }
  constexpr ReportDescriptor<TotalLength + 3> logical_max_i16(int16_t value) {
    return add_short_item_i16(ItemPrefix::LogicalMaximum, value);
  }
  constexpr ReportDescriptor<TotalLength + 5> logical_max_i32(int32_t value) {
    return add_short_item_i32(ItemPrefix::LogicalMaximum, value);
  }

  constexpr ReportDescriptor<TotalLength + 2> logical_max_u8(uint8_t value) {
    return add_short_item_u8(ItemPrefix::LogicalMaximum, value);
  }
  constexpr ReportDescriptor<TotalLength + 3> logical_max_u16(uint16_t value) {
    return add_short_item_u16(ItemPrefix::LogicalMaximum, value);
  }
  constexpr ReportDescriptor<TotalLength + 5> logical_max_u32(uint32_t value) {
    return add_short_item_u32(ItemPrefix::LogicalMaximum, value);
  }

  /**
   * Return a new descriptor with a PhysicalMinimum item appended.
   */
  constexpr ReportDescriptor<TotalLength + 2> physical_min(int8_t value) {
    return add_short_item_i8(ItemPrefix::PhysicalMinimum, value);
  }
  constexpr ReportDescriptor<TotalLength + 2> physical_min_i16(int16_t value) {
    return add_short_item_i16(ItemPrefix::PhysicalMinimum, value);
  }
  constexpr ReportDescriptor<TotalLength + 2> physical_min_i32(int32_t value) {
    return add_short_item_i32(ItemPrefix::PhysicalMinimum, value);
  }

  /**
   * Return a new descriptor with a PhysicalMaximum item appended.
   *
   * Note that PhysicalMaximum is treated as signed if the current
   * PhysicalMinimum state is negative, and unsigned otherwise.  (At least
   * according to the report parsing logic in the Linux kernel).
   */
  constexpr ReportDescriptor<TotalLength + 2> physical_max(int8_t value) {
    return add_short_item_i8(ItemPrefix::PhysicalMaximum, value);
  }
  constexpr ReportDescriptor<TotalLength + 3> physical_max_i16(int16_t value) {
    return add_short_item_i16(ItemPrefix::PhysicalMaximum, value);
  }
  constexpr ReportDescriptor<TotalLength + 5> physical_max_i32(int32_t value) {
    return add_short_item_i32(ItemPrefix::PhysicalMaximum, value);
  }

  constexpr ReportDescriptor<TotalLength + 2> physical_max_u8(uint8_t value) {
    return add_short_item_u8(ItemPrefix::PhysicalMaximum, value);
  }
  constexpr ReportDescriptor<TotalLength + 3> physical_max_u16(uint16_t value) {
    return add_short_item_u16(ItemPrefix::PhysicalMaximum, value);
  }
  constexpr ReportDescriptor<TotalLength + 5> physical_max_u32(uint32_t value) {
    return add_short_item_u32(ItemPrefix::PhysicalMaximum, value);
  }

  /**
   * Return a new descriptor with a UnitExponent item appended.
   */
  constexpr ReportDescriptor<TotalLength + 2> unit_exponent(int8_t exp) {
    return add_short_item_i8(ItemPrefix::UnitExponent, exp);
  }

  /**
   * Return a new descriptor with a Unit item appended.
   */
  constexpr ReportDescriptor<TotalLength + 2> unit_u8(uint8_t value) {
    return add_short_item_u8(ItemPrefix::Unit, value);
  }
  constexpr ReportDescriptor<TotalLength + 2> unit_u16(uint16_t value) {
    return add_short_item_u16(ItemPrefix::Unit, value);
  }
  constexpr ReportDescriptor<TotalLength + 2> unit_u32(uint32_t value) {
    return add_short_item_u32(ItemPrefix::Unit, value);
  }

  /**
   * Return a new descriptor with a ReportSize item appended.
   */
  constexpr ReportDescriptor<TotalLength + 2> report_size(uint8_t size) {
    return add_short_item_u8(ItemPrefix::ReportSize, size);
  }
  constexpr ReportDescriptor<TotalLength + 2> report_size_u16(uint16_t size) {
    return add_short_item_u16(ItemPrefix::ReportSize, size);
  }

  /**
   * Return a new descriptor with a ReportID item appended.
   */
  constexpr ReportDescriptor<TotalLength + 2> report_id(uint8_t id) {
    return add_short_item_u8(ItemPrefix::ReportID, id);
  }

  /**
   * Return a new descriptor with a ReportCount item appended.
   */
  constexpr ReportDescriptor<TotalLength + 2> report_count(uint8_t count) {
    return add_short_item_u8(ItemPrefix::ReportCount, count);
  }
  constexpr ReportDescriptor<TotalLength + 2> report_count_u16(uint16_t count) {
    return add_short_item_u16(ItemPrefix::ReportCount, count);
  }

  /**
   * Return a new descriptor with a Push item appended.
   */
  constexpr ReportDescriptor<TotalLength + 1> push() {
    return add_short_item(ItemPrefix::Push);
  }

  /**
   * Return a new descriptor with a Pop item appended.
   */
  constexpr ReportDescriptor<TotalLength + 1> pop() {
    return add_short_item(ItemPrefix::Pop);
  }

  /*************
   * Local Items
   *************/

  /**
   * Return a new descriptor with a Usage item appended.
   */
  constexpr ReportDescriptor<TotalLength + 2> usage(uint8_t value) {
    return add_short_item_u8(ItemPrefix::Usage, value);
  }
  constexpr ReportDescriptor<TotalLength + 3> usage_u16(uint16_t value) {
    return add_short_item_u16(ItemPrefix::Usage, value);
  }
  template <typename T>
  constexpr typename std::enable_if<is_usage_type<T>::value,
                                    ReportDescriptor<TotalLength + 2>>::type
  usage(T value) {
    return add_short_item_u8(ItemPrefix::Usage, static_cast<uint8_t>(value));
  }

  /**
   * Return a new descriptor with a UsageMinimum item appended.
   */
  template <typename T>
  constexpr typename std::enable_if<is_usage_type<T>::value,
                                    ReportDescriptor<TotalLength + 2>>::type
  usage_min(T value) {
    return add_short_item_u8(ItemPrefix::UsageMinimum,
                             static_cast<uint8_t>(value));
  }
  constexpr ReportDescriptor<TotalLength + 2> usage_min(uint8_t value) {
    return add_short_item_u8(ItemPrefix::UsageMinimum, value);
  }
  constexpr ReportDescriptor<TotalLength + 3> usage_min_u16(uint16_t value) {
    return add_short_item_u16(ItemPrefix::UsageMinimum, value);
  }
  constexpr ReportDescriptor<TotalLength + 5> usage_min_u32(uint32_t value) {
    return add_short_item_u32(ItemPrefix::UsageMinimum, value);
  }

  /**
   * Return a new descriptor with a UsageMinimum item appended.
   */
  template <typename T>
  constexpr typename std::enable_if<is_usage_type<T>::value,
                                    ReportDescriptor<TotalLength + 2>>::type
  usage_max(T value) {
    return add_short_item_u8(ItemPrefix::UsageMaximum,
                             static_cast<uint8_t>(value));
  }
  constexpr ReportDescriptor<TotalLength + 2> usage_max(uint8_t value) {
    return add_short_item_u8(ItemPrefix::UsageMaximum, value);
  }
  constexpr ReportDescriptor<TotalLength + 3> usage_max_u16(uint16_t value) {
    return add_short_item_u16(ItemPrefix::UsageMaximum, value);
  }
  constexpr ReportDescriptor<TotalLength + 5> usage_max_u32(uint32_t value) {
    return add_short_item_u32(ItemPrefix::UsageMaximum, value);
  }

  // TODO: DesignatorIndex = 0x38,
  // TODO: DesignatorMinimum = 0x48,
  // TODO: DesignatorMaximum = 0x58,
  // TODO: StringIndex = 0x78,
  // TODO: StringMinimum = 0x88,
  // TODO: StringMaximum = 0x98,
  // TODO: Delimiter = 0xa8,

  /*****************************
   * Generic Item helper methods
   *****************************/

  constexpr ReportDescriptor<TotalLength + 1>
  add_short_item(ItemPrefix prefix) {
    std::array<uint8_t, 1> item = {{static_cast<uint8_t>(prefix)}};
    return ReportDescriptor<TotalLength + 1>(*this, item.data());
  }

  constexpr ReportDescriptor<TotalLength + 2>
  add_short_item_u8(ItemPrefix prefix, uint8_t value) {
    std::array<uint8_t, 2> item = {{
        static_cast<uint8_t>(static_cast<uint8_t>(prefix) | 1),
        value,
    }};
    return ReportDescriptor<TotalLength + 2>(*this, item.data());
  }

  constexpr ReportDescriptor<TotalLength + 3>
  add_short_item_u16(ItemPrefix prefix, uint16_t value) {
    std::array<uint8_t, 3> item = {{
        static_cast<uint8_t>(static_cast<uint8_t>(prefix) | 2),
        static_cast<uint8_t>(value & 0xff),
        static_cast<uint8_t>((value >> 8) & 0xff),
    }};
    return ReportDescriptor<TotalLength + 3>(*this, item.data());
  }

  constexpr ReportDescriptor<TotalLength + 5>
  add_short_item_u32(ItemPrefix prefix, uint32_t value) {
    std::array<uint8_t, 5> item = {{
        static_cast<uint8_t>(static_cast<uint8_t>(prefix) | 3),
        static_cast<uint8_t>(value & 0xff),
        static_cast<uint8_t>((value >> 8) & 0xff),
        static_cast<uint8_t>((value >> 16) & 0xff),
        static_cast<uint8_t>((value >> 24) & 0xff),
    }};
    return ReportDescriptor<TotalLength + 5>(*this, item.data());
  }

  // Signed versions.  Whether or not an item's data is treated as signed or
  // unsigned appears to be based purely on context from the item type and
  // existing parsing state.  (e.g., Linux code for parsing HID report
  // descriptors treats the LogicalMaximum as unsigned if the current
  // LogicalMinimum state is greater than or equal to 0, and signed otherwise.)
  constexpr ReportDescriptor<TotalLength + 2>
  add_short_item_i8(ItemPrefix prefix, int8_t value) {
    return add_short_item_u8(prefix, static_cast<uint8_t>(value));
  }
  constexpr ReportDescriptor<TotalLength + 3>
  add_short_item_i16(ItemPrefix prefix, int16_t value) {
    return add_short_item_u16(prefix, static_cast<uint16_t>(value));
  }
  constexpr ReportDescriptor<TotalLength + 5>
  add_short_item_i32(ItemPrefix prefix, int32_t value) {
    return add_short_item_u32(prefix, static_cast<uint32_t>(value));
  }

private:
  template <size_t X>
  friend class ReportDescriptor;

  template <size_t OtherLength>
  constexpr ReportDescriptor(const ReportDescriptor<OtherLength> &other,
                             const uint8_t *data) {
    for (size_t n = 0; n < OtherLength; ++n) {
      data_[n] = other.data_[n];
    }
    for (size_t n = 0; OtherLength + n < TotalLength; ++n) {
      data_[OtherLength + n] = data[n];
    }
  }

  std::array<uint8_t, TotalLength> data_ = {};
};

} // namespace ausb::hid
