// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/hid/usage/keys.h"

#include <asel/array.h>
#include <asel/inttypes.h>

namespace ausb::kbd {

class BootReport {
public:
  constexpr BootReport() noexcept = default;

  void add_key(hid::Key key);

  bool is_rollover() const {
    return data_[2] == static_cast<uint8_t>(hid::Key::ErrorRollOver);
  }

  uint8_t *data() {
    return data_.data();
  }
  const uint8_t *data() const {
    return data_.data();
  }
  asel::array<uint8_t, 8> &array() {
    return data_;
  }
  const asel::array<uint8_t, 8> &array() const {
    return data_;
  }

  /**
   * Note: the == operator checks for exact equality.
   *
   * Two reports with identical sets of keys pressed but in different
   * order will compare as not equal, despite being logically equivalent.
   */
  bool operator==(const BootReport &other) {
    return data_ == other.data_;
  }
  bool operator!=(const BootReport &other) {
    return data_ != other.data_;
  }

private:
  asel::array<uint8_t, 8> data_ = {};
};

} // namespace ausb::kbd
