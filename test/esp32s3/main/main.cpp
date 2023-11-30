// Copyright (c) 2023, Adam Simpkins
#include <cstdio>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ausb/esp/Esp32Device.h"

using namespace ausb;
using namespace std::chrono_literals;

namespace {
static constinit Esp32Device dev;
const char *LogTag = "ausb.test";
}

[[nodiscard]] esp_err_t run_test() {
  ESP_LOGI(LogTag, "Starting USB initialization...");
  ESP_RETURN_ON_ERROR(dev.init(), LogTag, "Error initializing USB device.");
  ESP_LOGI(LogTag, "USB initialization complete.");

  size_t n = 0;
  while (true) {
    ++n;
    const auto event = dev.wait_for_event(10000ms);
    if (std::holds_alternative<UninitializedEvent>(event)) {
      ESP_LOGI(LogTag, "usb %zu: no event", n);
    } else {
      ESP_LOGI(LogTag, "usb %zu: got event", n);
    }
  }

  return ESP_OK;
}

extern "C" void app_main() {
  esp_log_level_set("ausb", ESP_LOG_VERBOSE);
  esp_log_level_set("ausb.esp32", ESP_LOG_VERBOSE);
  esp_log_level_set("ausb.test", ESP_LOG_VERBOSE);

  const auto err = run_test();
  if (err != ESP_OK) {
    // We could abort here, but for now it seems better to just wait rather
    // than resetting and probably just entering an error reset loop.
    while (true) {
      ESP_LOGE(LogTag, "error occurred: %d", err);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  ESP_LOGI(LogTag, "test complete");
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
