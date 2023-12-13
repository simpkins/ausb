// Copyright (c) 2023, Adam Simpkins
#include "asel/test/run_tests.h"

extern "C" int main() {
  const auto num_failures = asel::run_tests();
  return num_failures == 0 ? 0 : 1;
}
