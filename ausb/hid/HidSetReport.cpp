// Copyright (c) 2023, Adam Simpkins
#include "ausb/hid/HidSetReport.h"

#include "ausb/SetupPacket.h"
#include "ausb/hid/HidInterface.h"
#include "ausb/log.h"

namespace ausb::hid {

void HidSetReport::start(const SetupPacket &packet) {
  buf_.resize(packet.length);
  start_read(buf_.data(), packet.length);
}

void HidSetReport::out_data_received(uint32_t bytes_received) {
  if (intf_->set_report(buf_view(buf_.data(), buf_.size()))) {
    AUSB_LOGV("HID SET_REPORT succeeded");
    ack();
  } else {
    AUSB_LOGE("HID SET_REPORT processing failed");
    error();
  }
}

void HidSetReport::xfer_failed(XferFailReason reason) {}

} // namespace ausb::hid
