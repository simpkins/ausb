// Copyright (c) 2023, Adam Simpkins
#include <esp_log.h>

#include "ausb/UsbDevice.h"
#include "ausb/desc/ConfigDescriptor.h"
#include "ausb/desc/DeviceDescriptor.h"
#include "ausb/desc/EndpointDescriptor.h"
#include "ausb/desc/StaticDescriptorMap.h"
#include "ausb/dev/EndpointManager.h"
#include "ausb/hid/HidDescriptor.h"
#include "ausb/hid/KeyboardInterface.h"
#include "ausb/log.h"

using namespace ausb;
using namespace ausb::device;
using namespace std::chrono_literals;

namespace {
const char *LogTag = "ausb.test";

class TestDevice {
public:
  static constexpr uint8_t kConfigId = 1;
  static constexpr uint8_t kHidInEndpointNum = 1;

  bool set_configuration(uint8_t config_id, EndpointManager& ep_mgr) {
    if (config_id == 0) {
      ep_mgr.unconfigure();
      return true;
    }
    if (config_id != kConfigId) {
      return false;
    }

    auto res =
        ep_mgr.open_in_endpoint(kHidInEndpointNum,
                                &kbd_intf_.in_endpoint(),
                                EndpointType::Interrupt,
                                hid::KeyboardInterface::kDefaultMaxPacketSize);
    if (!res) {
      AUSB_LOGE("error opening HID IN endpoint");
    }

    ep_mgr.set_configured(config_id, &kbd_intf_);
    return true;
  }

  static constexpr auto make_descriptor_map() {
    DeviceDescriptor dev;
    dev.set_vendor(0x6666); // Prototype product vendor ID
    dev.set_product(0x1000);
    dev.set_device_release(1, 0);

    auto cfg = hid::KeyboardInterface::update_config_descriptor(
        ConfigDescriptor(kConfigId, ConfigAttr::RemoteWakeup),
        /*endpoint_num=*/1);

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

private:
  hid::KeyboardInterface kbd_intf_{kHidInEndpointNum};
};

static constinit UsbDevice<TestDevice> usb;

} // namespace

extern "C" void app_main() {
  esp_log_level_set("ausb", ESP_LOG_VERBOSE);
  esp_log_level_set("ausb.esp32", ESP_LOG_VERBOSE);
  esp_log_level_set("ausb.test", ESP_LOG_VERBOSE);

  ESP_LOGI(LogTag, "Starting USB initialization...");
  const auto init_err = usb.init();
  if (init_err) {
    ESP_LOGE(LogTag, "Error initializing USB device.");
    abort();
  }

  ESP_LOGI(LogTag, "Running USB task loop...");
  usb.loop();
}
