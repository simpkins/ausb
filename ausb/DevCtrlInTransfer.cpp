// Copyright (c) 2023, Adam Simpkins
#include "ausb/DevCtrlInTransfer.h"

#include "ausb/UsbDevice.h"

namespace ausb {

void DevCtrlInTransfer::send_full(const void* data, size_t size) {
  device_->start_ctrl_in_write(data, size);
}

void DevCtrlInTransfer::error() { device_->stall_ctrl_in_transfer(); }

} // namespace ausb
