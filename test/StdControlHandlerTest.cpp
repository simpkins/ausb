// Copyright (c) 2023, Adam Simpkins
#include "ausb/dev/StdControlHandler.h"

#include "ausb/desc/DeviceDescriptor.h"
#include "ausb/desc/StaticDescriptorMap.h"
#include "ausb/dev/EndpointManager.h"
#include "ausb/dev/EndpointZero.h"
#include "ausb/hw/mock/MockDevice.h"

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
  ep_mgr.handle_event(SuspendEvent{});
  ep_mgr.handle_event(BusResetEvent{});
  ep_mgr.handle_event(BusEnumDone{UsbSpeed::Low});
  ASEL_EXPECT_EQ(hw.out_eps[0].max_packet_size, 8);
  ASEL_EXPECT_EQ(hw.in_eps[0].max_packet_size, 8);

  SetupPacket get_dev_desc;
  get_dev_desc.request_type = 0x80; // IN, Device, Standard request
  get_dev_desc.request = 6;         // GET_DESCRIPTOR
  get_dev_desc.value = 0x0100;      // Device descriptor
  get_dev_desc.index = 0;
  get_dev_desc.length = 18;
  ep_mgr.handle_event(SetupPacketEvent{0, get_dev_desc});

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
  ep_mgr.handle_event(SuspendEvent{});
  ep_mgr.handle_event(BusResetEvent{});
  ep_mgr.handle_event(BusEnumDone{UsbSpeed::Full});
  ASEL_EXPECT_EQ(hw.out_eps[0].max_packet_size, 64);
  ASEL_EXPECT_EQ(hw.in_eps[0].max_packet_size, 64);

  SetupPacket get_dev_desc;
  get_dev_desc.request_type = 0x80; // IN, Device, Standard request
  get_dev_desc.request = 6;         // GET_DESCRIPTOR
  get_dev_desc.value = 0x0100;      // Device descriptor
  get_dev_desc.index = 0;
  get_dev_desc.length = 18;
  ep_mgr.handle_event(SetupPacketEvent{0, get_dev_desc});

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

} // namespace ausb::test
