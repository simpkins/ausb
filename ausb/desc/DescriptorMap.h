// Copyright (c) 2023, Adam Simpkins
#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

namespace ausb {

using buf_view = std::basic_string_view<uint8_t>;

class DescriptorMap {
public:
  constexpr DescriptorMap() noexcept = default;

  virtual std::optional<buf_view>
  get_descriptor_for_setup(uint16_t value, uint16_t index) const = 0;
};

} // namespace ausb
