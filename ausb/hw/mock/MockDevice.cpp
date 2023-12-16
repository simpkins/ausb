// Copyright (c) 2023, Adam Simpkins
#include "ausb/hw/mock/MockDevice.h"

#include <asel/test/checks.h>

namespace ausb {

std::error_code MockDevice::init() {
  return {};
}

void MockDevice::reset() {
  for (size_t n = 0; n < out_eps.size(); ++n) {
    out_eps[n].reset();
  }
  for (size_t n = 0; n < in_eps.size(); ++n) {
    in_eps[n].reset();
  }
}

DeviceEvent MockDevice::wait_for_event(std::chrono::milliseconds timeout) {
  return NoEvent{NoEventReason::Timeout};
}

void MockDevice::set_address(uint8_t address) {}

void MockDevice::set_address_early(uint8_t address) {}

bool MockDevice::configure_ep0(uint8_t max_packet_size) {
  if (out_eps[0].max_packet_size != 0 || in_eps[0].max_packet_size != 0) {
    return false;
  }
  out_eps[0].max_packet_size = max_packet_size;
  in_eps[0].max_packet_size = max_packet_size;
  return true;
}

bool MockDevice::open_in_endpoint(uint8_t endpoint_num,
                                  EndpointType type,
                                  uint16_t max_packet_size) {
  if (endpoint_num > in_eps.size()) {
    return false;
  }
  if (in_eps[endpoint_num].max_packet_size != 0) {
    // endpoint already open
    return false;
  }

  in_eps[endpoint_num].max_packet_size = max_packet_size;
  return true;
}

bool MockDevice::open_out_endpoint(uint8_t endpoint_num,
                                   EndpointType type,
                                   uint16_t max_packet_size) {
  if (endpoint_num > out_eps.size()) {
    return false;
  }
  if (out_eps[endpoint_num].max_packet_size != 0) {
    // endpoint already open
    return false;
  }

  out_eps[endpoint_num].max_packet_size = max_packet_size;
  return true;
}

XferStartResult
MockDevice::start_write(uint8_t endpoint, const void *data, uint32_t size) {
  if (endpoint >= in_eps.size()) {
    return XferStartResult::EndpointNotConfigured;
  }
  if (in_eps[endpoint].max_packet_size == 0) {
    return XferStartResult::EndpointNotConfigured;
  }
  if (in_eps[endpoint].xfer_in_progress) {
    return XferStartResult::Busy;
  }
  in_eps[endpoint].xfer_in_progress = true;
  in_eps[endpoint].cur_xfer_data = data;
  in_eps[endpoint].cur_xfer_size = size;
  return XferStartResult::Ok;
}

XferStartResult
MockDevice::start_read(uint8_t endpoint, void *data, uint32_t size) {
  if (endpoint >= out_eps.size()) {
    return XferStartResult::EndpointNotConfigured;
  }
  if (out_eps[endpoint].max_packet_size == 0) {
    return XferStartResult::EndpointNotConfigured;
  }
  if (out_eps[endpoint].xfer_in_progress) {
    return XferStartResult::Busy;
  }
  out_eps[endpoint].xfer_in_progress = true;
  out_eps[endpoint].cur_xfer_data = data;
  out_eps[endpoint].cur_xfer_size = size;
  return XferStartResult::Ok;
}

void MockDevice::stall_control_endpoint(uint8_t endpoint_num) {
  in_eps[endpoint_num].stalled = true;
  out_eps[endpoint_num].stalled = true;
}

DeviceEvent MockDevice::complete_in_xfer(uint8_t endpoint_num) {
  if (!ASEL_EXPECT_TRUE(in_eps[endpoint_num].xfer_in_progress)) {
    return NoEvent(NoEventReason::HwProcessing);
  }
  in_eps[endpoint_num].cur_xfer_data = nullptr;
  in_eps[endpoint_num].cur_xfer_size = 0;
  in_eps[endpoint_num].xfer_in_progress = false;
  return InXferCompleteEvent(endpoint_num);
}

DeviceEvent MockDevice::complete_out_xfer(uint8_t endpoint_num,
                                          int32_t bytes_read) {
  if (!ASEL_EXPECT_TRUE(out_eps[endpoint_num].xfer_in_progress)) {
    return NoEvent(NoEventReason::HwProcessing);
  }

  uint16_t const bytes_read_reply =
      bytes_read >= 0 ? bytes_read : out_eps[endpoint_num].cur_xfer_size;

  out_eps[endpoint_num].cur_xfer_data = nullptr;
  out_eps[endpoint_num].cur_xfer_size = 0;
  out_eps[endpoint_num].xfer_in_progress = false;
  return OutXferCompleteEvent(endpoint_num, bytes_read_reply);
}

void MockDevice::reset_in_stall(uint8_t endpoint_num) {
  in_eps[endpoint_num].stalled = false;
}

void MockDevice::reset_out_stall(uint8_t endpoint_num) {
  out_eps[endpoint_num].stalled = false;
}

} // namespace ausb
