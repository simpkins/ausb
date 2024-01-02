// Copyright (c) 2024, Adam Simpkins
#pragma once

#include <asel/array.h>
#include <asel/inttypes.h>

#include <cstring>

namespace ausb::kbd {

/**
 * A bitmap tracking which keys are currently pressed.
 */
template <size_t NumKeys, typename KeyTypeT = uint8_t>
class KeyBitmap {
public:
  using KeyType = KeyTypeT;
  using KeyIntType = size_t;
  static constexpr size_t kNumBytes = (NumKeys + 7) / 8;
  static constexpr size_t kNumU32s = (NumKeys + 31) / 32;

  class Changes;
  class Iterator;

  constexpr KeyBitmap() = default;

  bool get(KeyType index) const {
    return ((data_.u8[static_cast<KeyIntType>(index) >> 3] >>
             (static_cast<KeyIntType>(index) & 7)) &
            1);
  }
  void set(KeyType index, bool value) {
    if (value) {
      data_.u8[static_cast<KeyIntType>(index) >> 3] |=
          static_cast<uint8_t>(1 << (static_cast<KeyIntType>(index) & 7));
    } else {
      data_.u8[static_cast<KeyIntType>(index) >> 3] &=
          ~static_cast<uint8_t>(1 << (static_cast<KeyIntType>(index) & 7));
    }
  }

  void add_key(KeyType value) {
    set(value, true);
  }
  void clear() {
    memset(data_.u8.data(), 0, data_.u8.size());
  }

  Changes changes_from(const KeyBitmap<NumKeys, KeyTypeT> &other) const;

private:
  union {
    asel::array<uint8_t, kNumBytes> u8 = {};
    asel::array<uint32_t, kNumU32s> u32;
  } data_;
};

template <size_t NumKeys, typename KeyTypeT>
class KeyBitmap<NumKeys, KeyTypeT>::Changes {
public:
  using Bitmap = KeyBitmap<NumKeys, KeyTypeT>;

  Changes(const Bitmap *new_kb, const Bitmap *old_kb)
      : new_(new_kb), old_(old_kb) {}

  Iterator begin() const {
    return Iterator(new_, old_, Iterator::Begin);
  }
  Iterator end() const {
    return Iterator(new_, old_, Iterator::End);
  }

private:
  const Bitmap *new_;
  const Bitmap *old_;
};

template <size_t NumKeys, typename KeyTypeT>
class KeyBitmap<NumKeys, KeyTypeT>::Iterator {
public:
  using Bitmap = KeyBitmap<NumKeys, KeyTypeT>;
  using KeyType = KeyTypeT;
  using KeyIntType = size_t;

  enum BeginEnum { Begin };
  enum EndEnum { End };

  Iterator(const Bitmap *new_kb, const Bitmap *old_kb, BeginEnum)
      : new_(new_kb), old_(old_kb), position_(0) {
    advance_loop();
  }

  Iterator(const Bitmap *new_kb, const Bitmap *old_kb, EndEnum)
      : new_(new_kb), old_(old_kb), position_(NumKeys) {}

  bool operator==(const Iterator &other) const = default;
  bool operator!=(const Iterator &other) const = default;

  Iterator &operator++() {
    advance();
    return *this;
  }
  Iterator operator++(int) {
    Iterator tmp = *this;
    advance();
    return tmp;
  }

  const Iterator &operator*() const {
    return *this;
  }

  KeyType key() const {
    return static_cast<KeyType>(position_);
  }
  bool is_press() const {
    return new_->get(static_cast<KeyType>(position_));
  }

private:
  void advance() {
    ++position_;
    advance_loop();
  }

  void advance_loop() {
    size_t word_idx = position_ / 32;
    size_t byte_idx;
    size_t bit_idx;
    if ((position_ & 31) != 0) {
      byte_idx = (position_ >> 3);
      bit_idx = position_ & 7;
      if (bit_idx != 0) {
        goto bit_loop;
      }
      goto byte_loop;
    }

    // Optimize for the case when the keys are mostly the same between each
    // scan loop.  Check uint32_t values at a time.  If we find a word with
    // differences then check bytes at a time, and check individual bits once
    // we find specific bytes with differences.
    for (; word_idx < Bitmap::kNumU32s; ++word_idx) {
      if (new_->data_.u32[word_idx] != old_->data_.u32[word_idx]) [[unlikely]] {
        byte_idx = word_idx * 4;
      byte_loop:
        do {
          bit_idx = 0;
        bit_loop:
          const auto new_byte = new_->data_.u8[byte_idx];
          const auto old_byte = old_->data_.u8[byte_idx];
          if (new_byte != old_byte) {
            for (; bit_idx < 8; ++bit_idx) {
              if (((new_byte >> bit_idx) & 1) != ((old_byte >> bit_idx) & 1)) {
                position_ = (byte_idx * 8) + bit_idx;
                return;
              }
            }
          }
          ++byte_idx;
        } while ((byte_idx & 3) != 0);
      }
    }

    // No remaining differences
    position_ = NumKeys;
  }

  const Bitmap *new_;
  const Bitmap *old_;
  size_t position_;
};

template <size_t NumKeys, typename KeyTypeT>
typename KeyBitmap<NumKeys, KeyTypeT>::Changes
KeyBitmap<NumKeys, KeyTypeT>::changes_from(
    const KeyBitmap<NumKeys, KeyTypeT> &other) const {
  return Changes(this, &other);
}

} // namespace ausb::kbd
