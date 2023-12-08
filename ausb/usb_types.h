// Copyright (c) 2022, Adam Simpkins
#pragma once

#include <cstdint>

namespace ausb {

enum class Direction : uint8_t {
  Out = 0,
  In = 0x80,
};

/**
 * Endpoint type bits,
 * as used in the bmAttributes field of the endpoint descriptor.
 */
enum class EndpointType : uint8_t {
  Control = 0,
  Isochronous = 1,
  Bulk = 2,
  Interrupt = 3,
};

class EndpointNumber {
public:
  explicit constexpr EndpointNumber(uint8_t number) : number_(number) {}

  constexpr uint8_t value() const {
    return number_;
  }

private:
  uint8_t number_;
};

class EndpointAddress {
public:
  explicit constexpr EndpointAddress(uint8_t address) : address_(address) {}
  explicit constexpr EndpointAddress(EndpointNumber num, Direction dir)
      : address_(num.value() | static_cast<uint8_t>(dir)) {}

  constexpr Direction direction() const {
    return static_cast<Direction>(address_ & 0x80);
  }
  constexpr EndpointNumber number() const {
    return EndpointNumber(address_ & 0x7f);
  }

  constexpr uint8_t value() const {
    return address_;
  }

private:
  const uint8_t address_;
};

} // namespace ausb
