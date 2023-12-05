// Copyright (c) 2023, Adam Simpkins
#include "ausb/CtrlInXfer.h"

#include "ausb/UsbDevice.h"

namespace ausb::device {

void CtrlInXfer::send_full(const void* data, size_t size) {
  device_->start_ctrl_in_write(data, size);
}

void CtrlInXfer::error() { device_->stall_ctrl_in_transfer(); }

} // namespace ausb::device
