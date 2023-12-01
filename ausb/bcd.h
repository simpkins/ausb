// Copyright (c) 2023, Adam Simpkins
#pragma once

#include <cstdint>
#include <cstdlib>
#include <type_traits>

namespace ausb {

/**
 * BCD-encode an integer.
 *
 * This function is primarily intended to be used at compile time.
 * If passed an invalid value it will call abort(), which will result in a
 * compile failure when evaluated at compile time.
 */
constexpr uint8_t bcd_encode(uint8_t x) {
  if (x > 99) {
    // BCD encoding cannot represent values with more than 2 decimal digits
    if (std::is_constant_evaluated()) {
      // At compile time we can produce a compile error if invoked with bad
      // data
      abort();
    } else {
      // Just clamp the value if evaluated at runtime.
      return 0x99;
    }
  }
  const auto low_digit = x % 10;
  const auto high_digit = x / 10;
  return (high_digit << 4) | (low_digit);
}

constexpr uint8_t bcd_decode(uint8_t x) {
  const auto low = x & 0x0f;
  const auto high = (x >> 4) & 0x0f;
  if (std::is_constant_evaluated() && (low > 9 || high > 9)) {
    abort();
  }
  return (high * 10) + low;
}

} // namespace ausb
