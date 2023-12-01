// Copyright (c) 2022, Adam Simpkins
#pragma once

#include <cstdint>

namespace ausb {

enum class Direction : uint8_t {
  Out = 0,
  In = 0x80,
};

} // namespace ausb
