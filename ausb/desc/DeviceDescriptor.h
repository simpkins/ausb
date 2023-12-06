// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/bcd.h"
#include "ausb/desc/types.h"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <string_view>
#include <type_traits>
#include <utility>

namespace ausb {

using buf_view = std::basic_string_view<uint8_t>;

/**
 * A USB device descriptor.
 *
 * Note that we store and operate on device descriptors in serialized format.
 * In general the device descriptor is normally initialized once during setup,
 * and not manipulated much other than that, and we mostly need the serialized
 * form to return during GET_DESCRIPTOR requests.
 *
 * Note that the bMaxPacketSize0 field may need to be updated after bus
 * enumeration, since a different max packet size is required for different bus
 * sizes.  Therefore it may be necessary to store the device descriptor in
 * read-write memory rather than read-only memory.
 *
 * If you want to initialize a DeviceDescriptor mostly at compile time, it is
 * recommended to define a constexpr function that initializes and returns the
 * DeviceDescriptor.
 */
class DeviceDescriptor {
public:
  static constexpr size_t kSize = 18;
  static constexpr uint8_t kDefaultMfgrStrIdx = 1;
  static constexpr uint8_t kDefaultProductStrIdx = 2;
  static constexpr uint8_t kDefaultSerialStrIdx = 3;

  constexpr DeviceDescriptor()
      : data_{{
            kSize, // bLength
            static_cast<uint8_t>(DescriptorType::Device),
            0,                     // bcdUSB (lower half)
            2,                     // bcdUSB (upper half)
            0,                     // bDeviceClass
            0,                     // bDeviceSubClass
            0,                     // bDeviceProtocol
            64,                    // bMaxPacketSize0
            0x66,                  // idVendor (lower half)
            0x66,                  // idVendor (upper half)
            0,                     // idProduct (lower half)
            0,                     // idProduct (upper half)
            0,                     // bcdDevice (lower half)
            0,                     // bcdDevice (upper half)
            kDefaultMfgrStrIdx,    // iManufacturer
            kDefaultProductStrIdx, // iProduct
            kDefaultSerialStrIdx,  // iSerialNumber
            1,                     // bNumConfigurations
        }} {}

  constexpr const std::array<uint8_t, kSize> &data() const { return data_; }
  constexpr std::array<uint8_t, kSize> &data() { return data_; }

  constexpr DeviceDescriptor &set_class(UsbClass usb_class, uint8_t subclass,
                                        uint8_t protocol) {
    set_class(static_cast<uint8_t>(usb_class));
    set_subclass(subclass);
    set_protocol(protocol);
    return *this;
  }
  constexpr DeviceDescriptor &set_class(uint8_t usb_class, uint8_t subclass,
                                        uint8_t protocol) {
    set_class(usb_class);
    set_subclass(subclass);
    set_protocol(protocol);
    return *this;
  }
  constexpr DeviceDescriptor &set_class(UsbClass usb_class) {
    set_class(static_cast<uint8_t>(usb_class));
    return *this;
  }
  constexpr DeviceDescriptor &set_class(uint8_t usb_class) {
    data_[4] = usb_class;
    return *this;
  }
  constexpr uint8_t get_class() const { return data_[4]; }

  constexpr DeviceDescriptor &set_subclass(uint8_t subclass) {
    data_[5] = subclass;
    return *this;
  }
  constexpr uint8_t subclass() const { return data_[5]; }

  constexpr DeviceDescriptor &set_protocol(uint8_t protocol) {
    data_[6] = protocol;
    return *this;
  }
  constexpr uint8_t protocol() const { return data_[6]; }

  constexpr DeviceDescriptor &set_ep0_max_pkt_size(uint8_t mps) {
    data_[7] = mps;
    return *this;
  }
  constexpr uint8_t ep0_max_pkt_size() const { return data_[7]; }

  constexpr DeviceDescriptor& set_vendor(uint16_t vendor) {
    data_[8] = static_cast<uint8_t>(vendor & 0xff);
    data_[9] = static_cast<uint8_t>((vendor >> 8) & 0xff);
    return *this;
  }
  constexpr uint16_t vendor() const {
    return (static_cast<uint16_t>(data_[9]) << 8) |
           static_cast<uint16_t>(data_[8]);
  }
  constexpr DeviceDescriptor& set_product(uint16_t product) {
    data_[10] = static_cast<uint8_t>(product & 0xff);
    data_[11] = static_cast<uint8_t>((product >> 8) & 0xff);
    return *this;
  }
  constexpr DeviceDescriptor &set_product(uint16_t vendor, uint16_t product) {
    set_vendor(vendor);
    set_product(product);
    return *this;
  }
  constexpr uint16_t product() const {
    return (static_cast<uint16_t>(data_[11]) << 8) |
           static_cast<uint16_t>(data_[10]);
  }

  constexpr DeviceDescriptor& set_device_release(int major, int minor) {
    data_[12] = bcd_encode(minor);
    data_[13] = bcd_encode(major);
    return *this;
  }
  constexpr DeviceDescriptor& set_device_release_bcd(uint16_t release) {
    data_[12] = static_cast<uint8_t>(release & 0xff);
    data_[13] = static_cast<uint8_t>((release >> 8) & 0xff);
    return *this;
  }
  constexpr uint16_t device_release_bcd() const {
    return (static_cast<uint16_t>(data_[13]) << 8) |
           static_cast<uint16_t>(data_[12]);
  }
  constexpr std::pair<uint8_t, uint8_t> device_release() const {
    return {bcd_decode(data_[13]), bcd_decode(data_[12])};
  }

  constexpr DeviceDescriptor &set_mfgr_str_idx(uint8_t idx) {
    data_[14] = idx;
    return *this;
  }
  constexpr uint8_t mfgr_str_idx() const { return data_[14]; }
  constexpr DeviceDescriptor &set_product_str_idx(uint8_t idx) {
    data_[15] = idx;
    return *this;
  }
  constexpr uint8_t product_str_idx() const { return data_[15]; }
  constexpr DeviceDescriptor &set_serial_str_idx(uint8_t idx) {
    data_[16] = idx;
    return *this;
  }
  constexpr uint8_t serial_str_idx() const { return data_[16]; }

  constexpr DeviceDescriptor &set_num_configs(uint8_t num) {
    data_[17] = num;
    return *this;
  }
  constexpr uint8_t num_configs() const { return data_[17]; }

  constexpr DeviceDescriptor &set_usb_version(int major, int minor) {
    data_[2] = bcd_encode(minor);
    data_[3] = bcd_encode(major);
    return *this;
  }
  constexpr DeviceDescriptor &set_usb_version_bcd(uint16_t release) {
    data_[2] = static_cast<uint8_t>(release & 0xff);
    data_[3] = static_cast<uint8_t>((release >> 8) & 0xff);
    return *this;
  }
  constexpr uint16_t usb_version_bcd() const {
    return (static_cast<uint16_t>(data_[3]) << 8) |
           static_cast<uint16_t>(data_[2]);
  }
  constexpr std::pair<uint8_t, uint8_t> usb_version() const {
    return {bcd_decode(data_[3]), bcd_decode(data_[2])};
  }

private:
  std::array<uint8_t, kSize> data_ = {};
};

/**
 * A class for parsing a DeviceDescriptor data from an existing buffer.
 *
 * (This does unfortunately duplicate the accessor method code from
 * DeviceDescriptor, but in this case simply duplicating the methods seems
 * nicer to me than more complex alternatives that would avoid the
 * duplication.)
 */
class DeviceDescriptorParser {
public:
  static constexpr size_t kSize = 18;

  /**
   * Create a DeviceDescriptorParser.
   *
   * If the caller attempts to create a DeviceDescriptorParser with a buffer of
   * the incorrect size, the returned DeviceDescriptorParser will be invalid.
   * If the caller has not already verified the buffer length, valid() must be
   * called first before calling any other DeviceDescriptorParser methods.
   */
  constexpr DeviceDescriptorParser(const void *data, size_t size)
      : data_(size == kSize ? static_cast<const uint8_t *>(data) : nullptr) {
    if (std::is_constant_evaluated() && size != kSize) {
      abort();
    }
  }
  constexpr DeviceDescriptorParser(buf_view data)
      : data_(data.size() == kSize ? data.data() : nullptr) {
    if (std::is_constant_evaluated() && data.size() != kSize) {
      abort();
    }
  }

  constexpr bool valid() const { return data_ != nullptr; }
  constexpr explicit operator bool() const { return data_ != nullptr; }

  constexpr buf_view data() const { return buf_view(data_, kSize); }

  constexpr uint8_t get_class() const { return data_[4]; }
  constexpr uint8_t subclass() const { return data_[5]; }
  constexpr uint8_t protocol() const { return data_[6]; }
  constexpr uint8_t ep0_max_pkt_size() const { return data_[7]; }
  constexpr uint16_t vendor() const {
    return (static_cast<uint16_t>(data_[9]) << 8) |
           static_cast<uint16_t>(data_[8]);
  }
  constexpr uint16_t product() const {
    return (static_cast<uint16_t>(data_[11]) << 8) |
           static_cast<uint16_t>(data_[10]);
  }
  constexpr uint16_t device_release_bcd() const {
    return (static_cast<uint16_t>(data_[13]) << 8) |
           static_cast<uint16_t>(data_[12]);
  }
  constexpr std::pair<uint8_t, uint8_t> device_release() const {
    return {bcd_decode(data_[13]), bcd_decode(data_[12])};
  }

  constexpr uint8_t mfgr_str_idx() const { return data_[14]; }
  constexpr uint8_t product_str_idx() const { return data_[15]; }
  constexpr uint8_t serial_str_idx() const { return data_[16]; }

  constexpr uint8_t num_configs() const { return data_[17]; }
  constexpr uint16_t usb_version_bcd() const {
    return (static_cast<uint16_t>(data_[3]) << 8) |
           static_cast<uint16_t>(data_[2]);
  }
  constexpr std::pair<uint8_t, uint8_t> usb_version() const {
    return {bcd_decode(data_[3]), bcd_decode(data_[2])};
  }

private:
  const uint8_t* data_ = nullptr;
};

} // namespace ausb
