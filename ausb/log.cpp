// Copyright (c) 2023, Adam Simpkins
#include "ausb/log.h"

#include <cstdio>

namespace ausb {

#ifndef ESP_TARGET

// TODO: move logging code into the asel library, especially for supporting
// compiling with no stdlib
void log_message(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
  printf("\n");
}

#endif

} // namespace ausb
