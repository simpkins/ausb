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

  class Changes;
  class Iterator;

  constexpr KeyBitmap() = default;

  bool get(KeyType index) const {
    return ((data_[static_cast<KeyIntType>(index) >> 3] >>
             (static_cast<KeyIntType>(index) & 7)) &
            1);
  }
  void set(KeyType index, bool value) {
    if (value) {
      data_[static_cast<KeyIntType>(index) >> 3] |=
          static_cast<uint8_t>(1 << (static_cast<KeyIntType>(index) & 7));
    } else {
      data_[static_cast<KeyIntType>(index) >> 3] &=
          ~static_cast<uint8_t>(1 << (static_cast<KeyIntType>(index) & 7));
    }
  }

  void add_key(KeyType value) {
    set(value, true);
  }
  void clear() {
    memset(data_.data(), 0, data_.size());
  }

  Changes changes_from(const KeyBitmap<NumKeys, KeyTypeT> &other);

private:
  asel::array<uint8_t, kNumBytes> data_ = {};
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
    // TODO: read uint32 words at a time
    if (new_->get(static_cast<KeyType>(0)) ==
        old_->get(static_cast<KeyType>(0))) {
      advance();
    }
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
    // TODO: read uint32 words at a time
    ++position_;
    while (position_ < NumKeys) {
      if (new_->get(static_cast<KeyType>(position_)) !=
          old_->get(static_cast<KeyType>(position_))) {
        return;
      }
      ++position_;
    }
  }

  const Bitmap *new_;
  const Bitmap *old_;
  size_t position_;
};

template <size_t NumKeys, typename KeyTypeT>
typename KeyBitmap<NumKeys, KeyTypeT>::Changes
KeyBitmap<NumKeys, KeyTypeT>::changes_from(
    const KeyBitmap<NumKeys, KeyTypeT> &other) {
  return Changes(this, &other);
}

} // namespace ausb::kbd
