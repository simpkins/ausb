// Copyright (c) 2023, Adam Simpkins
#include "ausb/device/StdControlHandler.h"

#include "ausb/SetupPacket.h"
#include "ausb/UsbDevice.h"
#include "ausb/desc/DeviceDescriptor.h"
#include "ausb/desc/StaticDescriptorMap.h"
#include "ausb/device/EndpointManager.h"
#include "ausb/device/EndpointZero.h"
#include "ausb/hw/mock/MockDevice.h"
#include "test/lib/mock_utils.h"

#include <asel/test/checks.h>
#include <asel/test/TestCase.h>

using namespace ausb::device;

namespace ausb::test {

namespace {
constexpr auto make_descriptor_map() {
  DeviceDescriptor dev;
  dev.set_vendor(0xcafe);
  dev.set_product(0xd00d);
  dev.set_class(UsbClass::Hid);
  dev.set_subclass(1);
  dev.set_protocol(2);
  dev.set_device_release(12, 34);

  return StaticDescriptorMap().add_device_descriptor(dev);
}

const auto kDescriptors = make_descriptor_map();

class TestControlHandlerCallback : public StdControlHandlerCallback {
public:
  bool set_configuration(uint8_t config_id) override {
      return false;
  }
  std::optional<asel::buf_view> get_descriptor(uint16_t value,
                                               uint16_t index) override {
    return kDescriptors.get_descriptor_for_setup(value, index);
  }
};

} // namespace

ASEL_TEST(StdControlHandler, test_ep0_mps_low_speed) {
  MockDevice hw;
  TestControlHandlerCallback cb;

  StdControlHandler ctrl_handler(&cb);
  EndpointManager ep_mgr(&hw, &ctrl_handler);

  auto init_err = ep_mgr.init();
  ASEL_EXPECT_FALSE(init_err);

  // Test that endpoint 0's max packet size is reported as 8 bytes
  // when the bus is enumerated as low speed
  ep_mgr.on_suspend();
  ep_mgr.on_bus_reset();
  ep_mgr.on_enum_done(UsbSpeed::Low);
  ASEL_EXPECT_EQ(hw.out_eps[0].max_packet_size, 8);
  ASEL_EXPECT_EQ(hw.in_eps[0].max_packet_size, 8);

  SetupPacket get_dev_desc;
  get_dev_desc.request_type = 0x80; // IN, Device, Standard request
  get_dev_desc.request = 6;         // GET_DESCRIPTOR
  get_dev_desc.value = 0x0100;      // Device descriptor
  get_dev_desc.index = 0;
  get_dev_desc.length = 18;
  hw.setup_received(get_dev_desc);

  ASEL_ASSERT_TRUE(hw.in_eps[0].xfer_in_progress);
  buf_view reply(static_cast<const uint8_t *>(hw.in_eps[0].cur_xfer_data),
                 hw.in_eps[0].cur_xfer_size);
  // The EP0 max packet size in the reply should be set to 8 bytes,
  // since this is required when using low speed
  std::array<uint8_t, 18> expected_reply = {{
      18,   // length
      1,    // descriptor type: device
      0,    // USB minor version
      2,    // USB major version
      3,    // class
      1,    // subclass
      2,    // protocol
      8,    // max_packet size
      0xfe, // vendor (lower half)
      0xca, // vendor (upper half)
      0x0d, // product (lower half)
      0xd0, // product (upper half)
      0x34, // device version (lower half)
      0x12, // device version (upper half)
      1,    // manufacturer string index
      2,    // product string index
      3,    // serial string index
      1,    // num configurations
  }};
  ASEL_EXPECT_EQ(reply, expected_reply);
}

ASEL_TEST(StdControlHandler, test_ep0_mps_full_speed) {
  MockDevice hw;
  TestControlHandlerCallback cb;

  StdControlHandler ctrl_handler(&cb);
  EndpointManager ep_mgr(&hw, &ctrl_handler);

  auto init_err = ep_mgr.init();
  ASEL_EXPECT_FALSE(init_err);

  // Test that endpoint 0's max packet size is reported as 8 bytes
  // when the bus is enumerated as low speed
  ep_mgr.on_suspend();
  ep_mgr.on_bus_reset();
  ep_mgr.on_enum_done(UsbSpeed::Full);
  ASEL_EXPECT_EQ(hw.out_eps[0].max_packet_size, 64);
  ASEL_EXPECT_EQ(hw.in_eps[0].max_packet_size, 64);

  SetupPacket get_dev_desc;
  get_dev_desc.request_type = 0x80; // IN, Device, Standard request
  get_dev_desc.request = 6;         // GET_DESCRIPTOR
  get_dev_desc.value = 0x0100;      // Device descriptor
  get_dev_desc.index = 0;
  get_dev_desc.length = 18;
  hw.setup_received(get_dev_desc);

  ASEL_ASSERT_TRUE(hw.in_eps[0].xfer_in_progress);
  buf_view reply(static_cast<const uint8_t *>(hw.in_eps[0].cur_xfer_data),
                 hw.in_eps[0].cur_xfer_size);
  // The EP0 max packet size in the reply should be set to 64 bytes,
  // since this is required when using full speed
  std::array<uint8_t, 18> expected_reply = {{
      18,   // length
      1,    // descriptor type: device
      0,    // USB minor version
      2,    // USB major version
      3,    // class
      1,    // subclass
      2,    // protocol
      64,   // max_packet size
      0xfe, // vendor (lower half)
      0xca, // vendor (upper half)
      0x0d, // product (lower half)
      0xd0, // product (upper half)
      0x34, // device version (lower half)
      0x12, // device version (upper half)
      1,    // manufacturer string index
      2,    // product string index
      3,    // serial string index
      1,    // num configurations
  }};
  ASEL_EXPECT_EQ(reply, expected_reply);
}

namespace {
class MultiConfigDevice {
public:
  static constexpr uint8_t kConfigA = 0x12;
  static constexpr uint8_t kConfigB = 0x34;

  constexpr explicit MultiConfigDevice(EndpointManager *manager) {}

  bool set_configuration(uint8_t config_id, EndpointManager& ep_mgr) {
    if (config_id == 0) {
      ep_mgr.unconfigure();
      return true;
    }
    if (config_id == kConfigA || config_id == kConfigB) {
      // We call set_configured() with no interfaces, even though this isn't
      // a realistic configuration
      ep_mgr.set_configured(config_id, asel::range<Interface *const>{});
      return true;
    }
    return false;
  }

  static constexpr auto make_descriptor_map() {
    DeviceDescriptor dev;
    dev.set_vendor(0x1234);
    dev.set_product(0x5678);
    dev.set_device_release(1, 0);

    auto cfg_a = ConfigDescriptor(kConfigA, ConfigAttr::RemoteWakeup);
    auto cfg_b = ConfigDescriptor(kConfigB);

    return StaticDescriptorMap()
        .add_device_descriptor(dev)
        .add_language_ids(Language::English_US)
        .add_string(dev.mfgr_str_idx(), "ACME, Inc.", Language::English_US)
        .add_string(dev.product_str_idx(), "AUSB Test Device",
                    Language::English_US)
        .add_string(dev.serial_str_idx(), "00:00:00::00:00:00",
                    Language::English_US)
        .add_config_descriptor(cfg_a)
        .add_config_descriptor(cfg_b);
  }
};

constinit UsbDevice<MultiConfigDevice, MockDevice> multi_cfg_usb;
} // namespace

ASEL_TEST(StdControlHandler, multi_config) {
  attach_mock_device(multi_cfg_usb);

  // Make a GET_CONFIGURATION request
  uint8_t cfg_id;
  mock_send_get_config(multi_cfg_usb, cfg_id);
  // The currently selected config should be ConfigA
  ASEL_EXPECT_EQ(cfg_id, 0x12);

  // Send a SET_CONFIGURATION request to switch to ConfigB
  mock_send_set_config(multi_cfg_usb, 0x34);

  // GET_CONFIGURATION should now return ConfigB's ID
  mock_send_get_config(multi_cfg_usb, cfg_id);
  ASEL_EXPECT_EQ(cfg_id, 0x34);

  // Send a SET_CONFIGURATION request to unconfigure the device
  mock_send_set_config(multi_cfg_usb, 0);

  mock_send_get_config(multi_cfg_usb, cfg_id);
  ASEL_EXPECT_EQ(cfg_id, 0);

  // A SET_CONFIGURATION request with a bogus ID should fail and result in a
  // STALL
  SetupPacket set_cfg_78;
  set_cfg_78.request_type = 0x00; // Out, Device, Standard request
  set_cfg_78.request = 9;         // SET_CONFIGURATION
  set_cfg_78.value = 0x78;        // Config ID
  set_cfg_78.index = 0;
  set_cfg_78.length = 0;
  auto *const hw = multi_cfg_usb.hw();
  hw->setup_received(set_cfg_78);
  ASEL_EXPECT_TRUE(hw->in_eps[0].stalled);
  ASEL_EXPECT_TRUE(hw->out_eps[0].stalled);
  hw->reset_in_stall(0);
  hw->reset_out_stall(0);

  // GET_CONFIGURATION should still return config ID 0 now
  cfg_id = 0xff;
  mock_send_get_config(multi_cfg_usb, cfg_id);
  ASEL_EXPECT_EQ(cfg_id, 0);
}

} // namespace ausb::test
