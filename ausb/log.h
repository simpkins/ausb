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

#define AUSB_LOGV(arg, ...) printf(arg, ##__VA_ARGS__)
#define AUSB_LOGD(arg, ...) printf(arg, ##__VA_ARGS__)
#define AUSB_LOGI(arg, ...) printf(arg, ##__VA_ARGS__)
#define AUSB_LOGW(arg, ...) printf(arg, ##__VA_ARGS__)
#define AUSB_LOGE(arg, ...) printf(arg, ##__VA_ARGS__)

#endif

} // namespace ausb
