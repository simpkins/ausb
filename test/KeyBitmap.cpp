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

ASEL_TEST(KeyBitmap, iterator) {
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

} // namespace ausb::test
