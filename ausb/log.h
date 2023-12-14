// Copyright (c) 2023, Adam Simpkins
#pragma once

#ifdef ESP_TARGET
#include <esp_log.h>
#else
#include <cstdio>
#endif

namespace ausb {

#ifdef ESP_TARGET

#define AUSB_LOGV(arg, ...) ESP_LOGV("ausb", arg, ##__VA_ARGS__)
#define AUSB_LOGD(arg, ...) ESP_LOGD("ausb", arg, ##__VA_ARGS__)
#define AUSB_LOGI(arg, ...) ESP_LOGI("ausb", arg, ##__VA_ARGS__)
#define AUSB_LOGW(arg, ...) ESP_LOGW("ausb", arg, ##__VA_ARGS__)
#define AUSB_LOGE(arg, ...) ESP_LOGE("ausb", arg, ##__VA_ARGS__)

#else

#define AUSB_LOGV(arg, ...) AUSB_LOG_IMPL(10, arg, ##__VA_ARGS__)
#define AUSB_LOGD(arg, ...) AUSB_LOG_IMPL(20, arg, ##__VA_ARGS__)
#define AUSB_LOGI(arg, ...) AUSB_LOG_IMPL(30, arg, ##__VA_ARGS__)
#define AUSB_LOGW(arg, ...) AUSB_LOG_IMPL(40, arg, ##__VA_ARGS__)
#define AUSB_LOGE(arg, ...) AUSB_LOG_IMPL(50, arg, ##__VA_ARGS__)

#define AUSB_LOG_LEVEL 0

#define AUSB_LOG_IMPL(level, arg, ...)                                         \
  do {                                                                         \
    if ((level) >= AUSB_LOG_LEVEL) {                                           \
      ::ausb::log_message((arg), ##__VA_ARGS__);                               \
    }                                                                          \
  } while (0)

__attribute__((__format__ (__printf__, 1, 2)))
void log_message(const char* fmt, ...);

#endif

} // namespace ausb
