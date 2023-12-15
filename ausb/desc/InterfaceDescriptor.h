// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/desc/types.h"

#include <asel/buf_view.h>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <type_traits>

namespace ausb {

/**
 * A USB interface descriptor.
 */
class InterfaceDescriptor {
public:
  static constexpr size_t kSize = 9;

  constexpr InterfaceDescriptor(UsbClass usb_class = UsbClass::PerInterface,
                                uint8_t subclass = 0,
                                uint8_t protocol = 0)
      : data_{{
            kSize, // bLength
            static_cast<uint8_t>(DescriptorType::Interface),
            0,                               // bInterfaceNumber
            0,                               // bAlternateSetting
            0,                               // bNumEndpoints
            static_cast<uint8_t>(usb_class), // bInterfaceClass
            subclass,                        // bInterfaceSubClass
            protocol,                        // bInterfaceProtocol
            0,                               // iInterface
        }} {}

  constexpr const std::array<uint8_t, kSize> &data() const {
    return data_;
  }
  constexpr std::array<uint8_t, kSize> &data() {
    return data_;
  }

  constexpr InterfaceDescriptor &set_interface_number(uint8_t mps) {
    data_[2] = mps;
    return *this;
  }
  constexpr uint8_t interface_number() const {
    return data_[2];
  }

  constexpr InterfaceDescriptor &set_alt_setting(uint8_t value) {
    data_[3] = value;
    return *this;
  }
  constexpr uint8_t alt_setting() const {
    return data_[3];
  }

  constexpr InterfaceDescriptor &set_num_endpoints(uint8_t num) {
    data_[4] = num;
    return *this;
  }
  constexpr uint8_t num_endpoints() const {
    return data_[4];
  }

  constexpr InterfaceDescriptor &
  set_class(UsbClass usb_class, uint8_t subclass, uint8_t protocol) {
    set_class(static_cast<uint8_t>(usb_class));
    set_subclass(subclass);
    set_protocol(protocol);
    return *this;
  }
  constexpr InterfaceDescriptor &
  set_class(uint8_t usb_class, uint8_t subclass, uint8_t protocol) {
    set_class(usb_class);
    set_subclass(subclass);
    set_protocol(protocol);
    return *this;
  }
  constexpr InterfaceDescriptor &set_class(UsbClass usb_class) {
    set_class(static_cast<uint8_t>(usb_class));
    return *this;
  }
  constexpr InterfaceDescriptor &set_class(uint8_t usb_class) {
    data_[5] = usb_class;
    return *this;
  }
  constexpr uint8_t get_class() const {
    return data_[5];
  }

  constexpr InterfaceDescriptor &set_subclass(uint8_t subclass) {
    data_[6] = subclass;
    return *this;
  }
  constexpr uint8_t subclass() const {
    return data_[6];
  }

  constexpr InterfaceDescriptor &set_protocol(uint8_t protocol) {
    data_[7] = protocol;
    return *this;
  }
  constexpr uint8_t protocol() const {
    return data_[7];
  }

  // string index is the index of a string descriptor describing this
  // interface.
  constexpr InterfaceDescriptor &set_string_index(uint8_t index) {
    data_[8] = index;
    return *this;
  }
  constexpr uint8_t string_index() const {
    return data_[8];
  }

private:
  std::array<uint8_t, kSize> data_ = {};
};

/**
 * A class for parsing a InterfaceDescriptor data from an existing buffer.
 */
class InterfaceDescriptorParser {
public:
  static constexpr size_t kSize = 9;

  /**
   * Create an InterfaceDescriptorParser.
   *
   * If the caller attempts to create a InterfaceDescriptorParser with a buffer
   * of the incorrect size, the returned InterfaceDescriptorParser will be
   * invalid.  If the caller has not already verified the buffer length,
   * valid() must be called first before calling any other
   * InterfaceDescriptorParser methods.
   */
  constexpr InterfaceDescriptorParser(const void *data, size_t size)
      : data_(size == kSize ? static_cast<const uint8_t *>(data) : nullptr) {
    if (std::is_constant_evaluated() && size != kSize) {
      abort();
    }
  }
  constexpr InterfaceDescriptorParser(asel::buf_view data)
      : data_(data.size() == kSize ? data.data() : nullptr) {
    if (std::is_constant_evaluated() && data.size() != kSize) {
      abort();
    }
  }

  constexpr bool valid() const {
    return data_ != nullptr;
  }
  constexpr explicit operator bool() const {
    return data_ != nullptr;
  }

  constexpr asel::buf_view data() const {
    return asel::buf_view(data_, kSize);
  }

  constexpr uint8_t interface_number() const {
    return data_[2];
  }
  constexpr uint8_t alt_setting() const {
    return data_[3];
  }
  constexpr uint8_t num_endpoints() const {
    return data_[4];
  }
  constexpr uint8_t get_class() const {
    return data_[5];
  }
  constexpr uint8_t subclass() const {
    return data_[6];
  }
  constexpr uint8_t protocol() const {
    return data_[7];
  }
  constexpr uint8_t string_index() const {
    return data_[8];
  }

private:
  const uint8_t *data_ = nullptr;
};

} // namespace ausb
