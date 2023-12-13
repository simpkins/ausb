// Copyright (c) 2023, Adam Simpkins
#pragma once

#include <asel/buf_view.h>

#include <cstdint>
#include <optional>

namespace ausb {

class DescriptorMap {
public:
  constexpr DescriptorMap() noexcept = default;

  virtual std::optional<asel::buf_view>
  get_descriptor_for_setup(uint16_t value, uint16_t index) const = 0;
};

} // namespace ausb
