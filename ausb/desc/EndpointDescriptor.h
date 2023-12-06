// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/desc/types.h"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <string_view>
#include <type_traits>

namespace ausb {

using buf_view = std::basic_string_view<uint8_t>;

/**
 * A USB endpoint descriptor.
 */
class EndpointDescriptor {
public:
  static constexpr size_t kSize = 7;

  constexpr EndpointDescriptor(EndpointType type = EndpointType::Control,
                               Direction dir = Direction::Out,
                               uint8_t endpoint_num = 0)
      : data_{{
            kSize, // bLength
            static_cast<uint8_t>(DescriptorType::Endpoint),
            static_cast<uint8_t>(
                static_cast<uint8_t>(dir) |
                (endpoint_num &
                 static_cast<uint8_t>(0x7f))), // bEndpointAddress
            static_cast<uint8_t>(type),        // bmAttributes
            0,                                 // wMaxPacketSize (lower half)
            0,                                 // wMaxPacketSize (upper half)
            0,                                 // bInterval
        }} {}

  constexpr const std::array<uint8_t, kSize> &data() const { return data_; }
  constexpr std::array<uint8_t, kSize> &data() { return data_; }

  constexpr EndpointDescriptor &set_address(Direction dir,
                                            uint8_t endpoint_num) {
    data_[2] = (static_cast<uint8_t>(dir) | (endpoint_num & 0x7f));
    return *this;
  }
  constexpr EndpointDescriptor &set_address(uint8_t address) {
    data_[2] = address;
    return *this;
  }
  constexpr uint8_t address() const { return data_[2]; }
  constexpr uint8_t endpoint_number() const { return data_[2] & 0x7f; }
  constexpr Direction direction() const {
    return static_cast<Direction>(data_[2] & 0x80);
  }

  // Note that set_type() also sets the sync and usage fields of the
  // attributes, since these are tied to the type.
  constexpr EndpointDescriptor &
  set_type(EndpointType type, EndpointSync sync = EndpointSync::NoSync,
           EndpointUsage usage = EndpointUsage::Data) {
    return set_attributes(type, sync, usage);
  }
  constexpr EndpointDescriptor &
  set_attributes(EndpointType type, EndpointSync sync, EndpointUsage usage) {
    data_[3] = static_cast<uint8_t>(type) | static_cast<uint8_t>(sync) |
               static_cast<uint8_t>(usage);
    return *this;
  }
  constexpr EndpointType type() const {
    return static_cast<EndpointType>(data_[3] & 0x03);
  }
  constexpr EndpointSync sync_type() const {
    return static_cast<EndpointSync>(data_[3] & 0x0c);
  }
  constexpr EndpointUsage usage() const {
    return static_cast<EndpointUsage>(data_[3] & 0x30);
  }
  constexpr uint8_t attributes() const { return data_[3]; }

  constexpr EndpointDescriptor& set_max_packet_size(uint16_t mps) {
    data_[4] = static_cast<uint8_t>(mps & 0xff);
    data_[5] = static_cast<uint8_t>((mps >> 8) & 0xff);
    return *this;
  }
  constexpr uint16_t max_packet_size() const {
    return (static_cast<uint16_t>(data_[5]) << 8) |
           static_cast<uint16_t>(data_[4]);
  }

  constexpr EndpointDescriptor &set_interval(uint8_t interval) {
    data_[6] = interval;
    return *this;
  }
  constexpr uint8_t interval() const { return data_[6]; }

private:
  std::array<uint8_t, kSize> data_ = {};
};

/**
 * A class for parsing a EndpointDescriptor data from an existing buffer.
 */
class EndpointDescriptorParser {
public:
  static constexpr size_t kSize = 7;

  /**
   * Create an EndpointDescriptorParser.
   *
   * If the caller attempts to create a EndpointDescriptorParser with a buffer
   * of the incorrect size, the returned EndpointDescriptorParser will be
   * invalid.  If the caller has not already verified the buffer length,
   * valid() must be called first before calling any other
   * EndpointDescriptorParser methods.
   */
  constexpr EndpointDescriptorParser(const void *data, size_t size)
      : data_(size == kSize ? static_cast<const uint8_t *>(data) : nullptr) {
    if (std::is_constant_evaluated() && size != kSize) {
      abort();
    }
  }
  constexpr EndpointDescriptorParser(buf_view data)
      : data_(data.size() == kSize ? data.data() : nullptr) {
    if (std::is_constant_evaluated() && data.size() != kSize) {
      abort();
    }
  }

  constexpr bool valid() const { return data_ != nullptr; }
  constexpr explicit operator bool() const { return data_ != nullptr; }

  constexpr buf_view data() const { return buf_view(data_, kSize); }

  constexpr uint8_t address() const { return data_[2]; }
  constexpr uint8_t endpoint_number() const { return data_[2] & 0x7f; }
  constexpr Direction direction() const {
    return static_cast<Direction>(data_[2] & 0x80);
  }

  constexpr EndpointType type() const {
    return static_cast<EndpointType>(data_[3] & 0x03);
  }
  constexpr EndpointSync sync_type() const {
    return static_cast<EndpointSync>(data_[3] & 0x0c);
  }
  constexpr EndpointUsage usage() const {
    return static_cast<EndpointUsage>(data_[3] & 0x30);
  }
  constexpr uint8_t attributes() const { return data_[3]; }

  constexpr uint16_t max_packet_size() const {
    return (static_cast<uint16_t>(data_[5]) << 8) |
           static_cast<uint16_t>(data_[4]);
  }
  constexpr uint8_t interval() const { return data_[6]; }

private:
  const uint8_t* data_ = nullptr;
};

} // namespace ausb
