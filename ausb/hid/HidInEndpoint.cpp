// Copyright (c) 2023, Adam Simpkins
#include "ausb/hid/HidInEndpoint.h"

#include "ausb/dev/EndpointManager.h"
#include "ausb/log.h"

namespace ausb::hid {

void HidInEndpointImpl::start_xfer(const HidReportQueuePtr &queue,
                                   uint16_t longest_report_size) {
  // TODO
}

bool HidInEndpointImpl::is_xfer_in_progress() const {
  return xfer_in_progress_;
}

void HidInEndpointImpl::log_bad_report_id(uint8_t report_id) const {
  AUSB_LOGE("attempted to use unknown report ID %u on HID endpoint %u",
            report_id,
            endpoint_num_);
}

bool HidInEndpointImpl::on_in_xfer_complete() {
  if (send_zero_length_packet_) {
    // We need to send a 0-length packet to finish the current transfer
    send_zero_length_packet_ = false;
    manager_->start_in_write(endpoint_num_, nullptr, 0);
    return false;
  }

  xfer_in_progress_ = false;
  return true;
}

bool HidInEndpointImpl::on_in_xfer_failed(XferFailReason reason) {
  AUSB_LOGW("IN transfer failed on HID endpoint %u", endpoint_num_);
  send_zero_length_packet_ = false;
  xfer_in_progress_ = false;
  return true;
}

device::CtrlOutXfer *
HidInEndpointImpl::process_out_setup(device::MessagePipe *pipe,
                                     const SetupPacket &packet,
                                     HidReportMapIntf &report_map) {
  // TODO
  return nullptr;
}

device::CtrlInXfer *
HidInEndpointImpl::process_in_setup(device::MessagePipe *pipe,
                                    const SetupPacket &packet,
                                    HidReportMapIntf &report_map) {
  // TODO
  return nullptr;
}

} // namespace ausb::hid
