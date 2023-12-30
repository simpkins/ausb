// Copyright (c) 2023, Adam Simpkins
#include "ausb/hid/kbd/BootReport.h"

#include <asel/test/TestCase.h>
#include <asel/test/checks.h>

using namespace ausb::hid;
using namespace ausb::kbd;

namespace ausb::test {

ASEL_TEST(KbdBootReport, rollover) {
  BootReport report;
  ASEL_EXPECT_FALSE(report.is_rollover());
  report.add_key(Key::A);
  ASEL_EXPECT_FALSE(report.is_rollover());
  report.add_key(Key::B);
  report.add_key(Key::C);
  report.add_key(Key::D);
  report.add_key(Key::E);
  report.add_key(Key::F);
  ASEL_EXPECT_FALSE(report.is_rollover());
  report.add_key(Key::LeftControl);
  report.add_key(Key::RightGui);
  ASEL_EXPECT_FALSE(report.is_rollover());

  report.add_key(Key::G);
  ASEL_EXPECT_TRUE(report.is_rollover());
  std::array<uint8_t, 8> expected{{0x81, 0, 1, 1, 1, 1, 1, 1}};
  ASEL_EXPECT_EQ(expected, report.array());
}

} // namespace ausb::test
