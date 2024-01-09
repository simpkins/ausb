// Copyright (c) 2023, Adam Simpkins
#include "ausb/device/CtrlOutXfer.h"

#include "ausb/device/MessagePipe.h"

namespace ausb::device {

void CtrlOutXfer::start_read(void *data, uint32_t size) {
  pipe_->start_out_read(data, size);
}

void CtrlOutXfer::ack() {
  if (!pipe_) {
    // The transfer was already cancelled
    return;
  }
  pipe_->ack_out_xfer();
}

void CtrlOutXfer::error() {
  if (!pipe_) {
    // The transfer was already cancelled
    return;
  }
  pipe_->fail_out_xfer();
}

} // namespace ausb::device
