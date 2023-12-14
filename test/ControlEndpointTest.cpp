// Copyright (c) 2023, Adam Simpkins
#include "ausb/dev/ControlEndpoint.h"
#include "ausb/desc/DeviceDescriptor.h"
#include "ausb/desc/StaticDescriptorMap.h"
#include "ausb/dev/ControlHandler.h"
#include "ausb/dev/EndpointManager.h"
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

class TestControlHandlerCallback : public ControlHandlerCallback {
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

ASEL_TEST(ControlEndpoint, test_ep0_mps_low_speed) {
    MockDevice hw;
    TestControlHandlerCallback cb;

    ControlHandler ctrl_handler(&cb);
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
    get_dev_desc.request = 6;    // GET_DESCRIPTOR
    get_dev_desc.value = 0x0100; // Device descriptor
    get_dev_desc.index = 0;
    get_dev_desc.length = 18;
    ep_mgr.handle_event(SetupPacketEvent{0, get_dev_desc});

    ASEL_ASSERT_TRUE(hw.in_eps[0].xfer_in_progress);
    ASEL_EXPECT_EQ(hw.in_eps[0].cur_xfer_size, 18);
}

} // namespace ausb::test
