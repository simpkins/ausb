// Copyright (c) 2023, Adam Simpkins
#include "mock_utils.h"

#include "ausb/SetupPacket.h"
#include "ausb/desc/ConfigDescriptor.h"
#include "ausb/desc/DeviceDescriptor.h"
#include "ausb/device/EndpointManager.h"
#include "ausb/hw/mock/MockDevice.h"

#include <asel/test/checks.h>

using namespace ausb::device;

namespace ausb::test {

bool attach_mock_device(MockDevice *hw, EndpointManager *ep_manager) {
  hw->reset();
  const auto init_err = ep_manager->init();
  if (!ASEL_EXPECT_FALSE(init_err)) {
    return false;
  }

  ep_manager->on_suspend();
  ep_manager->on_bus_reset();
  ep_manager->on_enum_done(UsbSpeed::Full);

  // Walk through some normal steps that would be done by a host
  // Get the device descriptor
  SetupPacket get_dev_desc;
  get_dev_desc.request_type = 0x80; // IN, Device, Standard request
  get_dev_desc.request = 6;         // GET_DESCRIPTOR
  get_dev_desc.value = 0x0100;      // Device descriptor
  get_dev_desc.index = 0;
  get_dev_desc.length = 18;
  ep_manager->on_setup_received(0, get_dev_desc);
  if (!ASEL_EXPECT_TRUE(hw->in_eps[0].xfer_in_progress)) {
    return false;
  }
  buf_view dd_reply(
      static_cast<const uint8_t *>(hw->in_eps[0].cur_xfer_data),
      hw->in_eps[0].cur_xfer_size);
  if (!ASEL_EXPECT_EQ(dd_reply.size(), DeviceDescriptorParser::kSize)) {
    return false;
  }

  DeviceDescriptorParser dd(dd_reply);
  const auto num_configs = dd.num_configs();
  if (!ASEL_EXPECT_GE(num_configs, 1)) {
      return false;
  }

  hw->complete_in_xfer(0);
  if (!ASEL_EXPECT_TRUE(hw->out_eps[0].xfer_in_progress)) {
    return false;
  }
  ASEL_EXPECT_EQ(0, hw->out_eps[0].cur_xfer_size);
  hw->complete_out_xfer(0);

  // Call SET_ADDRESS
  SetupPacket set_addr;
  set_addr.request_type = 0x00; // Out, Device, Standard request
  set_addr.request = 5;         // SET_ADDRESS
  set_addr.value = 1234;        // Address
  set_addr.index = 0;
  set_addr.length = 0;
  hw->setup_received(set_addr);
  if (!ASEL_EXPECT_TRUE(hw->in_eps[0].xfer_in_progress)) {
    return false;
  }
  ASEL_EXPECT_EQ(0, hw->in_eps[0].cur_xfer_size);
  hw->complete_in_xfer(0);

  // Get the config descriptor
  SetupPacket get_cfg_desc;
  get_cfg_desc.request_type = 0x80; // IN, Device, Standard request
  get_cfg_desc.request = 6;         // GET_DESCRIPTOR
  get_cfg_desc.value = 0x0200;      // Config descriptor
  get_cfg_desc.index = 0;
  get_cfg_desc.length = 9;
  hw->setup_received(get_cfg_desc);
  if (!ASEL_EXPECT_TRUE(hw->in_eps[0].xfer_in_progress)) {
    return false;
  }
  buf_view cfg_reply(
      static_cast<const uint8_t *>(hw->in_eps[0].cur_xfer_data),
      hw->in_eps[0].cur_xfer_size);
  if (!ASEL_EXPECT_EQ(cfg_reply.size(), ConfigDescriptorParser::kSize)) {
    return false;
  }

  ConfigDescriptorParser cfg_desc(cfg_reply);
  const auto cfg_id = cfg_desc.value();

  hw->complete_in_xfer(0);
  if (!ASEL_EXPECT_TRUE(hw->out_eps[0].xfer_in_progress)) {
    return false;
  }
  ASEL_EXPECT_EQ(0, hw->out_eps[0].cur_xfer_size);
  hw->complete_out_xfer(0);

  // Call SET_CONFIGURATION with the first config ID
  if (!mock_send_set_config(hw, ep_manager, cfg_id)) {
    return false;
  }

  return true;
}

bool mock_send_set_config(MockDevice *hw, device::EndpointManager *ep_manager,
                          uint8_t config_id) {
  SetupPacket set_cfg;
  set_cfg.request_type = 0x00; // Out, Device, Standard request
  set_cfg.request = 9;         // SET_CONFIGURATION
  set_cfg.value = config_id;   // Config ID
  set_cfg.index = 0;
  set_cfg.length = 0;
  hw->setup_received(set_cfg);
  if (!ASEL_EXPECT_TRUE(hw->in_eps[0].xfer_in_progress)) {
    return false;
  }
  if (!ASEL_EXPECT_EQ(0, hw->in_eps[0].cur_xfer_size)) {
    return false;
  }
  hw->complete_in_xfer(0);

  return true;
}

bool mock_send_get_config(MockDevice *hw, device::EndpointManager *ep_manager,
                          uint8_t &config_id) {
  config_id = 0xff;

  SetupPacket set_cfg;
  set_cfg.request_type = 0x80; // IN, Device, Standard request
  set_cfg.request = 8;         // SET_CONFIGURATION
  set_cfg.value = 0;   // Config ID
  set_cfg.index = 0;
  set_cfg.length = 1;
  hw->setup_received(set_cfg);

  if (!ASEL_EXPECT_TRUE(hw->in_eps[0].xfer_in_progress)) {
    return false;
  }
  buf_view reply(
      static_cast<const uint8_t *>(hw->in_eps[0].cur_xfer_data),
      hw->in_eps[0].cur_xfer_size);
  if (!ASEL_EXPECT_EQ(reply.size(), 1)) {
    return false;
  }

  config_id = reply[0];

  // Ack the transfer
  hw->complete_in_xfer(0);
  if (!ASEL_EXPECT_TRUE(hw->out_eps[0].xfer_in_progress)) {
    return false;
  }
  if (!ASEL_EXPECT_EQ(0, hw->out_eps[0].cur_xfer_size)) {
    return false;
  }
  hw->complete_out_xfer(0);

  return true;
}

} // namespace ausb::test
