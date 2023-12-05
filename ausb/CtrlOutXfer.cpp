// Copyright (c) 2023, Adam Simpkins
#include "ausb/CtrlOutXfer.h"

#include "ausb/UsbDevice.h"

namespace ausb::device {

void CtrlOutXfer::start_read(void *data, uint32_t size) {
  device_->start_ctrl_out_read(data, size);
}

void CtrlOutXfer::ack() { device_->start_ctrl_in_write(0, 0); }

void CtrlOutXfer::error() { device_->stall_ctrl_out_transfer(); }

} // namespace ausb::device
