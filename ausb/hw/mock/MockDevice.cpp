// Copyright (c) 2023, Adam Simpkins
#include "ausb/hw/mock/MockDevice.h"

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

XferStartResult MockDevice::start_write(uint8_t endpoint, const void *data,
                                        uint32_t size) {
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

XferStartResult MockDevice::start_read(uint8_t endpoint, void *data,
                                       uint32_t size) {
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

void MockDevice::stall_control_endpoint(uint8_t endpoint_num) {}

} // namespace ausb
