// Copyright (c) 2024, Adam Simpkins
#include "ausb/hid/kbd/KeyBitmap.h"

namespace ausb::kbd::detail {

size_t key_bitmap_advance_iterator(const uint32_t *new_u32,
                                   const uint32_t *old_u32,
                                   size_t start_position,
                                   size_t num_keys) {
  // Note: we have to be a little careful about C++'s strict aliasing rules
  // here.  We do store new_u32 and old_u32 in a union of both uint32_t and
  // uint8_t arrays, so we could have the caller pass in both the u32 and u8
  // pointers.  However, passing in just one pointer is more convenient and
  // results in fewer function arguments.
  //
  // In general casting one pointer type to another and then dereferencing it
  // is disallowed since it can cause aliasing that would prevent compiler
  // optimizations.  However, it is explicitly allowed to access any data type
  // through a character pointer, so we are allowed to cast to unsigned char*
  // here.
  const auto *new_u8 = reinterpret_cast<const unsigned char *>(new_u32);
  const auto *old_u8 = reinterpret_cast<const unsigned char *>(old_u32);
  const size_t num_u32s = (num_keys + 31) / 32;

  size_t word_idx = start_position / 32;
  size_t byte_idx;
  size_t bit_idx;
  if ((start_position & 31) != 0) {
    byte_idx = (start_position >> 3);
    bit_idx = start_position & 7;
    if (bit_idx != 0) {
      goto bit_loop;
    }
    goto byte_loop;
  }

  // Optimize for the case when the keys are mostly the same between each
  // scan loop.  Check uint32_t values at a time.  If we find a word with
  // differences then check bytes at a time, and check individual bits once
  // we find specific bytes with differences.
  for (; word_idx < num_u32s; ++word_idx) {
    if (new_u32[word_idx] !=
        (old_u32 ? old_u32[word_idx] : static_cast<uint32_t>(0))) [[unlikely]] {
      byte_idx = word_idx * 4;
    byte_loop:
      do {
        bit_idx = 0;
      bit_loop:
        const auto new_byte = new_u8[byte_idx];
        const auto old_byte =
            old_u32 ? old_u8[byte_idx] : static_cast<uint8_t>(0);
        if (new_byte != old_byte) {
          for (; bit_idx < 8; ++bit_idx) {
            if (((new_byte >> bit_idx) & 1) != ((old_byte >> bit_idx) & 1))
                [[unlikely]] {
              return (byte_idx * 8) + bit_idx;
            }
          }
        }
        ++byte_idx;
      } while ((byte_idx & 3) != 0);
    }
  }

  // No remaining differences
  return num_keys;
}

} // namespace ausb::kbd::detail
