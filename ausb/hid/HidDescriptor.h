// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/bcd.h"
#include "ausb/desc/types.h"
#include "ausb/hid/types.h"

#include <asel/buf_view.h>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <type_traits>

namespace ausb {

/**
 * A USB HID descriptor.
 */
class HidDescriptor {
public:
  static constexpr size_t kSize = 9;
  static constexpr size_t kTotalLength = kSize;

  constexpr HidDescriptor()
      : data_{{
            kSize, // bLength
            static_cast<uint8_t>(DescriptorType::Hid),
            0x11, // bcdHID (lower half)
            0x01, // bcdHID (upper half)
            0,    // bCountryCode
            0,    // bNumDescriptors
            0,    // bDescriptorType
            0,    // wDescriptorLength (lower half)
            0     // wDescriptorLength (upper half)
        }} {}

  constexpr const std::array<uint8_t, kSize> &data() const { return data_; }
  constexpr std::array<uint8_t, kSize> &data() { return data_; }

  constexpr HidDescriptor &set_country(HidCountry country) {
    data_[4] = static_cast<uint8_t>(country);
    return *this;
  }
  constexpr HidCountry country() const {
    return static_cast<HidCountry>(data_[4]);
  }

  constexpr HidDescriptor &set_num_descriptors(uint8_t idx) {
    data_[5] = idx;
    return *this;
  }
  constexpr uint8_t num_descriptors() const { return data_[5]; }

  constexpr HidDescriptor &set_report_descriptor_type(DescriptorType type) {
    data_[6] = static_cast<uint8_t>(type);
    return *this;
  }
  constexpr DescriptorType report_descriptor_type() const {
    return static_cast<DescriptorType>(data_[6]);
  }

  constexpr HidDescriptor& set_report_descriptor_length(uint16_t length) {
    data_[7] = static_cast<uint8_t>(length & 0xff);
    data_[8] = static_cast<uint8_t>((length >> 8) & 0xff);
    return *this;
  }
  constexpr uint16_t report_descriptor_length() const {
    return (static_cast<uint16_t>(data_[8]) << 8) |
           static_cast<uint16_t>(data_[7]);
  }

  constexpr HidDescriptor &set_hid_version(int major, int minor) {
    data_[2] = bcd_encode(minor);
    data_[3] = bcd_encode(major);
    return *this;
  }
  constexpr HidDescriptor &set_hid_version_bcd(uint16_t release) {
    data_[2] = static_cast<uint8_t>(release & 0xff);
    data_[3] = static_cast<uint8_t>((release >> 8) & 0xff);
    return *this;
  }
  constexpr uint16_t hid_version_bcd() const {
    return (static_cast<uint16_t>(data_[3]) << 8) |
           static_cast<uint16_t>(data_[2]);
  }
  constexpr std::pair<uint8_t, uint8_t> hid_version() const {
    return {bcd_decode(data_[3]), bcd_decode(data_[2])};
  }

private:
  std::array<uint8_t, kSize> data_ = {};
};

/**
 * A class for parsing a HidDescriptor data from an existing buffer.
 */
class HidDescriptorParser {
public:
  static constexpr size_t kSize = 9;

  /**
   * Create a HidDescriptorParser.
   *
   * If the caller attempts to create a HidDescriptorParser with a buffer
   * of the incorrect size, the returned HidDescriptorParser will be
   * invalid.  If the caller has not already verified the buffer length,
   * valid() must be called first before calling any other
   * HidDescriptorParser methods.
   */
  constexpr HidDescriptorParser(const void *data, size_t size)
      : data_(size == kSize ? static_cast<const uint8_t *>(data) : nullptr) {
    if (std::is_constant_evaluated() && size != kSize) {
      abort();
    }
  }
  constexpr HidDescriptorParser(asel::buf_view data)
      : data_(data.size() == kSize ? data.data() : nullptr) {
    if (std::is_constant_evaluated() && data.size() != kSize) {
      abort();
    }
  }

  constexpr bool valid() const { return data_ != nullptr; }
  constexpr explicit operator bool() const { return data_ != nullptr; }

  constexpr asel::buf_view data() const { return asel::buf_view(data_, kSize); }

  // TODO: accessor methods

private:
  const uint8_t* data_ = nullptr;
};

} // namespace ausb
