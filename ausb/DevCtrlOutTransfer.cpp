// Copyright (c) 2023, Adam Simpkins
#include "ausb/DevCtrlOutTransfer.h"

#include "ausb/UsbDevice.h"

namespace ausb {

void DevCtrlOutTransfer::start_read(void *data, uint32_t size) {
  device_->start_ctrl_out_read(data, size);
}

void DevCtrlOutTransfer::ack() { device_->start_ctrl_in_write(0, 0); }

void DevCtrlOutTransfer::error() { device_->stall_ctrl_out_transfer(); }

} // namespace ausb
