// Copyright (c) 2023, Adam Simpkins
#include "ausb/desc/StaticDescriptorMapUtils.h"

namespace ausb::detail {

std::optional<buf_view>
get_usb_descriptor(uint16_t value,
                    uint16_t index,
                    const uint8_t *data,
                    size_t data_size,
                    const StaticDescriptorMapEntry *entries,
                    size_t num_entries) {
  // Do a simple linear scan of all entries.
  //
  // We could sort the entries during construction, but there should usually be
  // few enough entries that a linear scan is probably just as fast as trying
  // to do a smarter search algorithm.
  for (size_t n = 0; n < num_entries; ++n) {
    if (entries[n].value == value && entries[n].index == index) {
      // TODO: handle ExternalDescriptorPtr entries
      return buf_view(data + entries[n].offset, entries[n].length);
    }
  }

  return std::nullopt;
}


} // namespace ausb
