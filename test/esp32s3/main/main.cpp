// Copyright (c) 2023, Adam Simpkins
#include <cstdio>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ausb/UsbDevice.h"
#include "ausb/desc/ConfigDescriptor.h"
#include "ausb/desc/DeviceDescriptor.h"
#include "ausb/desc/EndpointDescriptor.h"
#include "ausb/desc/StaticDescriptorMap.h"
#include "ausb/dev/ControlHandler.h"
#include "ausb/hw/esp/Esp32Device.h"

using namespace ausb;
using namespace ausb::device;
using namespace std::chrono_literals;

namespace {
constexpr auto make_descriptor_map() {
  DeviceDescriptor dev;
  dev.set_vendor(0x6666); // Prototype product vendor ID
  dev.set_product(0x1235);
  dev.set_device_release(0, 1);

  EndpointDescriptor ep1;
  ep1.set_address(Direction::In, 1);
  ep1.set_type(EndpointType::Interrupt);
  ep1.set_interval(10);
  ep1.set_max_packet_size(8);

  auto cfg = ConfigDescriptor(1, ConfigAttr::RemoteWakeup)
                 .add_interface(InterfaceDescriptor(UsbClass::Hid))
                 .add_endpoint(ep1);

  return StaticDescriptorMap()
      .add_device_descriptor(dev)
      .add_language_ids(Language::English_US)
      .add_string(dev.mfgr_str_idx(), "Adam Simpkins", Language::English_US)
      .add_string(dev.product_str_idx(), "AUSB Test Device",
                  Language::English_US)
      .add_string(dev.serial_str_idx(), "00:00:00::00:00:00",
                  Language::English_US)
      .add_config_descriptor(cfg);
}

static constexpr auto kDescriptors = make_descriptor_map();
static constinit Esp32Device dev;
ControlHandler ctrl_handler(&kDescriptors);
static constinit UsbDevice usb(&dev, &ctrl_handler);
const char *LogTag = "ausb.test";

void dump_hex(const uint8_t* buf, uint16_t size) {
  auto p = buf;
  size_t bytes_left = size;
  while (bytes_left > 8) {
    printf("- %02x %02x %02x %02x %02x %02x %02x %02x\n",
           p[0],
           p[1],
           p[2],
           p[3],
           p[4],
           p[5],
           p[6],
           p[7]);
    p += 8;
    bytes_left -= 8;
  }
  if (bytes_left > 0) {
    printf("-");
    while (bytes_left > 0) {
      printf(" %02x", p[0]);
      ++p;
      --bytes_left;
    }
    printf("\n");
  }
}

void dump_desc(uint16_t value, uint16_t index) {
  printf("Descriptor %#x  %#x:\n", value, index);
  auto desc = kDescriptors.get_descriptor_with_setup_ids(value, index);
  if (!desc.has_value()) {
    printf("- none\n");
    return;
  }

  printf("- size: %d\n", desc->size());
  dump_hex(desc->data(), desc->size());
}

void dump_descriptors() {
  printf("USB Descriptors:\n");
  dump_desc(0x100, 0);
  dump_desc(0x200, 0);
  dump_desc(0x300, 0);
  dump_desc(0x301, 0x0409);
  dump_desc(0x302, 0x0409);
  dump_desc(0x303, 0x0409);
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

} // namespace

extern "C" void app_main() {
  esp_log_level_set("ausb", ESP_LOG_VERBOSE);
  esp_log_level_set("ausb.esp32", ESP_LOG_VERBOSE);
  esp_log_level_set("ausb.test", ESP_LOG_VERBOSE);

  ESP_LOGI(LogTag, "test starting.  sizeof(kDescriptors)=%zu",
           sizeof(kDescriptors));

  const auto desc = kDescriptors.get_descriptor(DescriptorType::Device);
  ESP_LOGI(LogTag, "device descriptor size: %zu", desc ? desc->size() : 0);

  dump_descriptors();

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
