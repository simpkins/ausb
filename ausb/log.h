// Copyright (c) 2023, Adam Simpkins
#pragma once

#include <esp_log.h>

namespace ausb {

#define AUSB_LOGI(arg, ...) ESP_LOGI("ausb", arg, ##__VA_ARGS__)
#define AUSB_LOGW(arg, ...) ESP_LOGW("ausb", arg, ##__VA_ARGS__)

} // namespace ausb
