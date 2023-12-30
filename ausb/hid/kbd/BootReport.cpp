// Copyright (c) 2023, Adam Simpkins
#include "ausb/hid/kbd/BootReport.h"

namespace ausb::kbd {

void BootReport::add_key(hid::Key key) {
  auto const value = static_cast<uint8_t>(key);
  if (value >= 0xe0 && value <= 0xe7) {
    data_[0] |= 1 << (value - 0xe0);
    return;
  }

  for (unsigned int idx = 2; idx < data_.size(); ++idx) {
    if (data_[idx] == 0) {
      data_[idx] = value;
      return;
    }
  }

  // Rollover.  Ensure bytes 2 through 8 are all set to Key::ErrorRollOver
  if (data_[2] == 1) {
    return;
  }
  for (unsigned int idx = 2; idx < data_.size(); ++idx) {
    data_[idx] = 1;
  }
}

} // namespace ausb::kbd
