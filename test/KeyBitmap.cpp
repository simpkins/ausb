// Copyright (c) 2024, Adam Simpkins
#include "ausb/hid/kbd/KeyBitmap.h"

#include "ausb/hid/usage/keys.h"

#include <asel/test/TestCase.h>
#include <asel/test/checks.h>

using namespace ausb::hid;
using namespace ausb::kbd;

namespace ausb::test {

struct KeyDiff {
  hid::Key key = hid::Key::None;
  bool is_press = false;

  bool operator==(const KeyDiff &) const = default;
  bool operator!=(const KeyDiff &) const = default;
};

void log_field(const KeyDiff &diff) {
  fprintf(stderr,
          "Diff(%zu, %s)",
          static_cast<size_t>(diff.key),
          diff.is_press ? "press" : "release");
}

ASEL_TEST(KeyBitmap, changes_from_hid) {
  using KB = KeyBitmap<256, hid::Key>;
  KB a;
  KB b;

  a.add_key(Key::A);
  b.add_key(Key::A);

  a.add_key(Key::B);
  b.add_key(Key::C);

  b.add_key(Key::R);
  a.add_key(Key::W);

  asel::array<KeyDiff, 5> diffs;
  size_t diff_idx = 0;
  for (auto change : a.changes_from(b)) {
    diffs[diff_idx].key = change.key();
    diffs[diff_idx].is_press = change.is_press();
    ++diff_idx;
    if (diff_idx >= diffs.size()) {
      break;
    }
  }

  asel::array<KeyDiff, 5> expected;
  expected[0].key = hid::Key::B;
  expected[0].is_press = true;
  expected[1].key = hid::Key::C;
  expected[1].is_press = false;
  expected[2].key = hid::Key::R;
  expected[2].is_press = false;
  expected[3].key = hid::Key::W;
  expected[3].is_press = true;
  ASEL_EXPECT_EQ(diffs, expected);
}

template <size_t NumKeys, typename KeyTypeT, typename... Args>
void expect_diff(typename KeyBitmap<NumKeys, KeyTypeT>::Iterator iter,
                 const typename KeyBitmap<NumKeys, KeyTypeT>::Iterator &end) {
  if (iter != end) {
    ASEL_ADD_FAILURE("unexpected key change (",
                     iter.key(),
                     ", ",
                     iter.is_press() ? "press" : "release",
                     ")");
    return;
  }
}

template <size_t NumKeys, typename KeyTypeT, typename... Args>
void expect_diff(typename KeyBitmap<NumKeys, KeyTypeT>::Iterator iter,
                 const typename KeyBitmap<NumKeys, KeyTypeT>::Iterator &end,
                 KeyTypeT key,
                 bool is_press,
                 Args &&...args) {
  if (iter == end) {
    ASEL_ADD_FAILURE("missing key change: (",
                     key,
                     ", ",
                     is_press ? "press" : "release",
                     ")");
    return;
  }
  if (iter.key() != key) {
    if (key < iter.key()) {
      ASEL_ADD_FAILURE("missing key change (",
                       key,
                       ", ",
                       is_press ? "press" : "release",
                       ")");
      expect_diff<NumKeys, KeyTypeT>(iter, end, std::forward<Args>(args)...);
      return;
    } else {
      ASEL_ADD_FAILURE("unexpected key change (",
                       iter.key(),
                       ", ",
                       iter.is_press() ? "press" : "release",
                       ")");
      expect_diff<NumKeys, KeyTypeT>(
          ++iter, end, key, is_press, std::forward<Args>(args)...);
      return;
    }
  }

  if (iter.is_press() != is_press) {
    ASEL_ADD_FAILURE("unexpected press type for key ",
                     key,
                     ": expected ",
                     is_press ? "press" : "release",
                     ", found ",
                     iter.is_press() ? "press" : "release");
  }
  expect_diff<NumKeys, KeyTypeT>(++iter, end, std::forward<Args>(args)...);
}

template <size_t NumKeys, typename KeyTypeT, typename... Args>
void expect_diff(const KeyBitmap<NumKeys, KeyTypeT> &new_kb,
                 const KeyBitmap<NumKeys, KeyTypeT> &old_kb,
                 Args &&...args) {
  auto changes = new_kb.changes_from(old_kb);
  expect_diff<NumKeys, KeyTypeT>(
      changes.begin(), changes.end(), std::forward<Args>(args)...);
}

ASEL_TEST(KeyBitmap, changes_from) {
  using KB = KeyBitmap<231, uint8_t>;
  KB a;
  KB b;
  a.add_key(7);
  b.add_key(8);
  a.add_key(64);
  b.add_key(125);
  a.add_key(127);
  b.add_key(128);

  expect_diff(
      a, b, 7, true, 8, false, 64, true, 125, false, 127, true, 128, false);
}

} // namespace ausb::test
