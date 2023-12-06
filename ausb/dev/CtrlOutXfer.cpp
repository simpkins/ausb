// Copyright (c) 2023, Adam Simpkins
#include "ausb/dev/CtrlOutXfer.h"

#include "ausb/dev/ControlEndpoint.h"

namespace ausb::device {

void CtrlOutXfer::start_read(void *data, uint32_t size) {
  endpoint_->start_out_read(data, size);
}

void CtrlOutXfer::ack() {
  if (!endpoint_) {
    // The transfer was already cancelled
    return;
  }
  endpoint_->ack_out_xfer();
}

void CtrlOutXfer::error() {
  if (!endpoint_) {
    // The transfer was already cancelled
    return;
  }
  endpoint_->fail_out_xfer();
}

} // namespace ausb::device
