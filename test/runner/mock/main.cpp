// Copyright (c) 2023, Adam Simpkins
#include <cstdio>

namespace ausb {
void descriptor_test();
}

extern "C" int main() {
  printf("starting\n");
  ausb::descriptor_test();
  printf("done\n");
  return 0;
}
