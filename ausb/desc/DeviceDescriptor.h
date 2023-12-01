// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/bcd.h"
#include "ausb/desc/types.h"

#include <array>
#include <cstdint>
#include <utility>

namespace ausb {

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

  constexpr void set_class(UsbClass usb_class, uint8_t subclass,
                           uint8_t protocol) {
    set_class(static_cast<uint8_t>(usb_class));
    set_subclass(subclass);
    set_protocol(protocol);
  }
  constexpr void set_class(uint8_t usb_class, uint8_t subclass,
                           uint8_t protocol) {
    set_class(usb_class);
    set_subclass(subclass);
    set_protocol(protocol);
  }
  constexpr void set_class(UsbClass usb_class) {
    set_class(static_cast<uint8_t>(usb_class));
  }
  constexpr void set_class(uint8_t usb_class) { data_[4] = usb_class; }
  constexpr uint8_t get_class() const { return data_[4]; }

  constexpr void set_subclass(uint8_t subclass) { data_[5] = subclass; }
  constexpr uint8_t subclass() const { return data_[5]; }

  constexpr void set_protocol(uint8_t subclass) { data_[6] = subclass; }
  constexpr uint8_t protocol() const { return data_[6]; }

  constexpr void set_max_pkt_size0(uint8_t mps) { data_[7] = mps; }
  constexpr uint8_t max_pkt_size0() const { return data_[7]; }

  constexpr void set_vendor(uint16_t vendor) {
    data_[8] = static_cast<uint8_t>(vendor & 0xff);
    data_[9] = static_cast<uint8_t>((vendor >> 8) & 0xff);
  }
  constexpr uint16_t vendor() const {
    return (static_cast<uint16_t>(data_[9]) << 8) |
           static_cast<uint16_t>(data_[8]);
  }
  constexpr void set_product(uint16_t product) {
    data_[10] = static_cast<uint8_t>(product & 0xff);
    data_[11] = static_cast<uint8_t>((product >> 8) & 0xff);
  }
  constexpr void set_product(uint16_t vendor, uint16_t product) {
    set_vendor(vendor);
    set_product(product);
  }
  constexpr uint16_t product() const {
    return (static_cast<uint16_t>(data_[11]) << 8) |
           static_cast<uint16_t>(data_[10]);
  }

  constexpr void set_device_release(int major, int minor) {
    data_[12] = bcd_encode(minor);
    data_[13] = bcd_encode(major);
  }
  constexpr void set_device_release_bcd(uint16_t release) {
    data_[12] = static_cast<uint8_t>(release & 0xff);
    data_[13] = static_cast<uint8_t>((release >> 8) & 0xff);
  }
  constexpr uint16_t device_release_bcd() const {
    return (static_cast<uint16_t>(data_[13]) << 8) |
           static_cast<uint16_t>(data_[12]);
  }
  constexpr std::pair<uint8_t, uint8_t> device_release() const {
    return {bcd_decode(data_[13]), bcd_decode(data_[12])};
  }

  constexpr void set_mfgr_str_idx(uint8_t idx) { data_[14] = idx; }
  constexpr uint8_t mfgr_str_idx() const { return data_[14]; }
  constexpr void set_product_str_idx(uint8_t idx) { data_[15] = idx; }
  constexpr uint8_t product_str_idx() const { return data_[15]; }
  constexpr void set_serial_str_idx(uint8_t idx) { data_[16] = idx; }
  constexpr uint8_t serial_str_idx() const { return data_[16]; }

  constexpr void set_num_configs(uint8_t num) { data_[17] = num; }
  constexpr uint8_t num_configs() const { return data_[17]; }

  constexpr void set_usb_version(int major, int minor) {
    data_[2] = bcd_encode(minor);
    data_[3] = bcd_encode(major);
  }
  constexpr void set_usb_version_bcd(uint16_t release) {
    data_[2] = static_cast<uint8_t>(release & 0xff);
    data_[3] = static_cast<uint8_t>((release >> 8) & 0xff);
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

} // namespace ausb
