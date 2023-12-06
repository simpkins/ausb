// Copyright (c) 2023, Adam Simpkins
#include <cstdio>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ausb/ControlHandler.h"
#include "ausb/UsbDevice.h"
#include "ausb/desc/DeviceDescriptor.h"
#include "ausb/esp/Esp32Device.h"

using namespace ausb;
using namespace ausb::device;
using namespace std::chrono_literals;

namespace {
constexpr DeviceDescriptor make_device_descriptor() {
  DeviceDescriptor dev;
  dev.set_vendor(0x6666); // Prototype product vendor ID
  dev.set_product(0x1235);
  dev.set_device_release(0, 1);
  return dev;
}

static constinit Esp32Device dev;
ControlHandler ctrl_handler(make_device_descriptor());
static constinit UsbDevice usb(&dev, &ctrl_handler);
const char *LogTag = "ausb.test";
}

[[nodiscard]] std::error_code run_test() {
  ESP_LOGI(LogTag, "Starting USB initialization...");
  const auto init_err = usb.init();
  if (init_err) {
      ESP_LOGE(LogTag, "Error initializing USB device.");
      return init_err;
  }
  ESP_LOGI(LogTag, "USB initialization complete.");

  size_t n = 0;
  while (true) {
    ++n;
    const auto event = dev.wait_for_event(10000ms);
    if (std::holds_alternative<NoEvent>(event)) {
      ESP_LOGI(LogTag, "usb %zu: no event", n);
    } else {
      ESP_LOGI(LogTag, "usb %zu: got event", n);
      usb.handle_event(event);
    }
  }

  return std::error_code();
}

extern "C" void app_main() {
  esp_log_level_set("ausb", ESP_LOG_VERBOSE);
  esp_log_level_set("ausb.esp32", ESP_LOG_VERBOSE);
  esp_log_level_set("ausb.test", ESP_LOG_VERBOSE);

  const auto err = run_test();
  if (err) {
    // We could abort here, but for now it seems better to just wait rather
    // than resetting and probably just entering an error reset loop.
    while (true) {
      ESP_LOGE(LogTag, "error occurred: %s", err.message().c_str());
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  ESP_LOGI(LogTag, "test complete");
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
