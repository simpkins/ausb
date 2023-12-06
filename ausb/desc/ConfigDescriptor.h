// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/desc/types.h"
#include "ausb/desc/EndpointDescriptor.h"
#include "ausb/desc/InterfaceDescriptor.h"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <string_view>
#include <type_traits>

namespace ausb {

using buf_view = std::basic_string_view<uint8_t>;

/**
 * A USB configuration descriptor.
 *
 * This class is primarily intended to be used to construct ConfigDescriptors
 * at compile time.  See ConfigDescriptorParser for parsing config descriptor
 * data from an existing buffer, which is more useful for runtime descriptor
 * parsing.
 *
 * Note that config descriptors are variable length, as they contain the
 * initial configuration descriptor data, followed by the interface and
 * endpoint descriptors.  This class is templatized on the overall data length
 * so that we can construct arbitrary length config descriptors at compile
 * time.
 */
template <size_t TotalLength = 9, uint8_t NumInterfaces = 0>
class ConfigDescriptor {
public:
  static constexpr size_t kSize = 9;
  static constexpr size_t kTotalLength = TotalLength;
  static constexpr size_t kNumInterfaces = NumInterfaces;

  constexpr ConfigDescriptor(uint8_t value = 1,
                             ConfigAttr attr = ConfigAttr::None,
                             UsbMilliamps max_power = UsbMilliamps(50),
                             uint8_t string_index = 0)
      : data_{{
            kSize, // bLength
            static_cast<uint8_t>(DescriptorType::Config),
            kTotalLength & 0xff,        // wTotalLength (lower half)
            (kTotalLength >> 8) & 0xff, // wTotalLength (upper half)
            kNumInterfaces,             // bNumInterfaces
            value,                      // bConfigurationValue
            string_index,               // iConfiguration
            static_cast<uint8_t>(attr), // bmAttributes
            max_power.value_in_2ma(),   // bMaxPower
        }} {}

  constexpr const std::array<uint8_t, kTotalLength> &data() const {
    return data_;
  }
  constexpr std::array<uint8_t, TotalLength> &data() { return data_; }

  /**
   * Return a new ConfigDescriptor object created by appending
   * a new interface descriptor.
   *
   * Note that this method does not modify the current object, but instead
   * returns a new, larger object.
   *
   * Also note that this method will ignore the bInterfaceNumber and
   * bNumEndpoints fields in the supplied interface descriptor, and will
   * automatically set the correct values.  (bNumEndpoints will be
   * automatically updated in future calls to add_endpoint().)
   * Use add_interface_raw() if you wish to use the InterfaceDescriptor
   * contents exactly as supplied, without automatically updating these fields.
   *
   * The bNumInterfaces field in the returned config descriptor will
   * automatically be incremented by 1.
   */
  constexpr ConfigDescriptor<TotalLength + InterfaceDescriptor::kSize,
                             NumInterfaces + 1>
  add_interface(const InterfaceDescriptor &intf) {
    return ConfigDescriptor<TotalLength + InterfaceDescriptor::kSize,
                            NumInterfaces + 1>(
        *this,
        // Rather than passing in the caller's InterfaceDescriptor, pass in a
        // copy that has the bInterfaceNumber field set to the correct value.
        InterfaceDescriptor(intf).set_interface_number(kNumInterfaces),
        TotalLength);
  }

  /**
   * Return a new ConfigDescriptor object created by appending
   * a new interface descriptor.
   *
   * Unlike add_interface(), this version does not automatically update the
   * bInterfaceNumber and bNumEndpoints fields in the appended interface
   * descriptor.
   *
   * The bNumInterfaces field in the returned ConfigDescriptor is still
   * incremented by one.  set_num_iterfaces() can be called if for some reason
   * you want to explicitly set the num interfaces field.
   */
  constexpr ConfigDescriptor<TotalLength + InterfaceDescriptor::kSize,
                             NumInterfaces + 1>
  add_interface_raw(const InterfaceDescriptor &intf) {
    return ConfigDescriptor<TotalLength + InterfaceDescriptor::kSize,
                            NumInterfaces + 1>(*this, intf, 0);
  }

  /**
   * Return a new ConfigDescriptor object created by appending
   * a new endpoint descriptor.
   *
   * Note that this method does not modify the current object, but instead
   * returns a new, larger object.
   */
  constexpr ConfigDescriptor<TotalLength + EndpointDescriptor::kSize,
                             NumInterfaces>
  add_endpoint(const EndpointDescriptor &endpoint) {
    return ConfigDescriptor<TotalLength + EndpointDescriptor::kSize,
                            NumInterfaces>(*this, endpoint);
  }

  constexpr uint16_t total_length() const {
    return (static_cast<uint16_t>(data_[3]) << 8) |
           static_cast<uint16_t>(data_[2]);
  }
  constexpr uint8_t num_interfaces() const { return data_[4]; }

  /**
   * Explicitly set the bNumInterfaces field.
   *
   * Note that this method should not be necessary in most cases: the
   * add_interface() method automatically increments bNumInterfaces in the
   * returned ConfigDescriptor.
   */
  constexpr ConfigDescriptor &set_num_interfaces(uint8_t num) {
    data_[4] = num;
    return *this;
  }

  constexpr ConfigDescriptor &set_value(uint8_t v) {
    data_[5] = v;
    return *this;
  }
  constexpr uint8_t value() const { return data_[5]; }

  // string index is the index of a string descriptor describing this
  // configuration.
  constexpr ConfigDescriptor &set_string_index(uint8_t index) {
    data_[6] = index;
    return *this;
  }
  constexpr uint8_t string_index() const { return data_[6]; }

  constexpr ConfigDescriptor& set_attributes(ConfigAttr attr) {
    data_[7] = static_cast<uint8_t>(attr);
    return *this;
  }
  constexpr ConfigAttr attributes() const {
    return static_cast<ConfigAttr>(data_[6]);
  }
  constexpr ConfigDescriptor &set_attributes_u8(uint8_t attr) {
    data_[7] = attr;
    return *this;
  }
  constexpr uint8_t attributes_u8() const { return data_[7]; }

  constexpr ConfigDescriptor& set_max_power(UsbMilliamps ma) {
    data_[8] = ma.value_in_2ma();
  }
  constexpr UsbMilliamps max_power() const {
    return UsbMilliamps(data_[8] * 2);
  }
  constexpr ConfigDescriptor &set_max_power_2ma(uint8_t power_2ma) {
    data_[8] = power_2ma;
    return *this;
  }
  constexpr uint8_t max_power_2ma() const { return data_[8]; }

private:
  template <size_t X, uint8_t Y> friend class ConfigDescriptor;

  // Constructor for appending an InterfaceDescriptor to an existing
  // ConfigDescriptor
  constexpr ConfigDescriptor(
      const ConfigDescriptor<TotalLength - InterfaceDescriptor::kSize,
                             NumInterfaces - 1> &other,
      const InterfaceDescriptor &intf, uint32_t last_intf_offset)
      : last_intf_offset_(last_intf_offset) {
    static_assert(TotalLength <= std::numeric_limits<uint16_t>::max(),
                  "config descriptor data is to large");
    // Manually set the initial few bytes, since this includes the wTotalLength
    // and bNumInterfaces fields which need to be updated.
    data_[0] = other.data_[0];
    data_[1] = other.data_[1];
    data_[2] = kTotalLength & 0xff;
    data_[3] = (kTotalLength >> 8) & 0xff;
    data_[4] = kNumInterfaces;
    constexpr size_t other_size = TotalLength - InterfaceDescriptor::kSize;
    for (size_t n = 5; n < other_size; ++n) {
      data_[n] = other.data_[n];
    }
    for (size_t n = 0; n < intf.data().size(); ++n) {
      data_[other_size + n] = intf.data()[n];
    }
  }

  // Constructor for appending an EndpointDescriptor to an existing
  // ConfigDescriptor
  constexpr ConfigDescriptor(
      const ConfigDescriptor<TotalLength - EndpointDescriptor::kSize,
                             NumInterfaces> &other,
      const EndpointDescriptor &endpoint)
      : last_intf_offset_(other.last_intf_offset_) {
    static_assert(TotalLength <= std::numeric_limits<uint16_t>::max(),
                  "config descriptor data is to large");
    data_[0] = other.data_[0];
    data_[1] = other.data_[1];
    data_[2] = kTotalLength & 0xff;
    data_[3] = (kTotalLength >> 8) & 0xff;
    constexpr size_t other_size = TotalLength - EndpointDescriptor::kSize;
    for (size_t n = 4; n < other_size; ++n) {
      data_[n] = other.data_[n];
    }
    for (size_t n = 0; n < endpoint.data().size(); ++n) {
      data_[other_size + n] = endpoint.data()[n];
    }

    if (last_intf_offset_ != 0) {
      // Increment the bNumEndpoints field in the interface descriptor
      data_[last_intf_offset_ + 4] += 1;
    }
  }

  std::array<uint8_t, TotalLength> data_ = {};
  // The offset to the last InterfaceDescriptor
  // This is used by add_endpoint() to automatically increment the
  // bNumEndpoints field in the relevant interface descriptor.
  uint32_t last_intf_offset_ = 0;
};

/**
 * A class for parsing a ConfigDescriptor data from an existing buffer.
 */
class ConfigDescriptorParser {
public:
  static constexpr size_t kSize = 18;

  /**
   * Create a ConfigDescriptorParser.
   *
   * If the caller attempts to create a ConfigDescriptorParser with a buffer
   * that is too small, the returned ConfigDescriptorParser will be invalid.
   * If the caller has not already verified the buffer length, valid() must be
   * called first before calling any other ConfigDescriptorParser methods.
   */
  constexpr ConfigDescriptorParser(const void *data, size_t size)
      : data_(size < kSize ? static_cast<const uint8_t *>(data) : nullptr),
        size_(size) {
    if (std::is_constant_evaluated() && size != kSize) {
      abort();
    }
  }
  constexpr ConfigDescriptorParser(buf_view data)
      : data_(data.size() < kSize ? data.data() : nullptr), size_(data.size()) {
    if (std::is_constant_evaluated() && data.size() != kSize) {
      abort();
    }
  }

  constexpr bool valid() const { return data_ != nullptr; }
  constexpr explicit operator bool() const { return data_ != nullptr; }

  constexpr buf_view data() const { return buf_view(data_, size_); }

  constexpr uint16_t total_length() const {
    return (static_cast<uint16_t>(data_[2]) << 8) |
           static_cast<uint16_t>(data_[3]);
  }

  constexpr uint8_t num_interfaces() const { return data_[4]; }
  constexpr uint8_t value() const { return data_[5]; }
  constexpr uint8_t string_index() const { return data_[6]; }

  constexpr ConfigAttr attributes() const {
    return static_cast<ConfigAttr>(data_[6]);
  }
  constexpr uint8_t attributes_u8() const { return data_[7]; }

  constexpr UsbMilliamps max_power() const {
    return UsbMilliamps(data_[8] * 2);
  }
  constexpr uint8_t max_power_2ma() const { return data_[8]; }

private:
  const uint8_t* data_ = nullptr;
  size_t size_ = 0;
};

} // namespace ausb
