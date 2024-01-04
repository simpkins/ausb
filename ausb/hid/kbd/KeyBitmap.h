// Copyright (c) 2024, Adam Simpkins
#pragma once

#include <asel/array.h>
#include <asel/inttypes.h>

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
    for (size_t n = 0; n < data_.u32.size(); ++n) {
      data_.u32[n] = 0;
    }
  }

  Changes changes_from(const KeyBitmap<NumKeys, KeyTypeT> &other) const;
  bool has_changes(const KeyBitmap<NumKeys, KeyTypeT> &other) const {
    for (size_t n = 0; n < data_.u32.size(); ++n) {
      if (data_.u32[n] != other.data_.u32[n]) {
        return true;
      }
    }
    return false;
  }

  Changes pressed_keys() const;
  bool any_pressed() const {
    for (size_t n = 0; n < data_.u32.size(); ++n) {
      if (data_.u32[n] != 0) {
        return true;
      }
    }
    return false;
  }

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

namespace detail {
size_t key_bitmap_advance_iterator(const uint32_t *new_u32,
                                   const uint32_t *old_u32,
                                   size_t start_position,
                                   size_t num_keys);
}

template <size_t NumKeys, typename KeyTypeT>
class KeyBitmap<NumKeys, KeyTypeT>::Iterator {
public:
  using Bitmap = KeyBitmap<NumKeys, KeyTypeT>;
  using KeyType = KeyTypeT;
  using KeyIntType = size_t;

  enum BeginEnum { Begin };
  enum EndEnum { End };

  Iterator(const Bitmap *new_kb, const Bitmap *old_kb, BeginEnum)
      : new_(new_kb),
        old_(old_kb),
        position_(detail::key_bitmap_advance_iterator(
            new_->data_.u32.data(),
            old_ ? old_->data_.u32.data() : nullptr,
            0,
            NumKeys)) {}

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
    position_ = detail::key_bitmap_advance_iterator(
        new_->data_.u32.data(),
        old_ ? old_->data_.u32.data() : nullptr,
        position_ + 1,
        NumKeys);
  }

  const Bitmap *new_ = nullptr;
  const Bitmap *old_ = nullptr;
  size_t position_ = 0;
};

template <size_t NumKeys, typename KeyTypeT>
typename KeyBitmap<NumKeys, KeyTypeT>::Changes
KeyBitmap<NumKeys, KeyTypeT>::changes_from(
    const KeyBitmap<NumKeys, KeyTypeT> &other) const {
  return Changes(this, &other);
}

template <size_t NumKeys, typename KeyTypeT>
typename KeyBitmap<NumKeys, KeyTypeT>::Changes
KeyBitmap<NumKeys, KeyTypeT>::pressed_keys() const {
  return Changes(this, nullptr);
}

} // namespace ausb::kbd
