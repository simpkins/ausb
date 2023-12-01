// Copyright (c) 2023, Adam Simpkins
#include "ausb/DevCtrlInTransfer.h"

#include "ausb/UsbDevice.h"

namespace ausb {

void DevCtrlInTransfer::send_full(const void* data, size_t size) {
  device_->send_ctrl_in_xfer(data, size);
}

} // namespace ausb
