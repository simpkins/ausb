// Copyright (c) 2023, Adam Simpkins
#include "ausb/dev/ctrl/StallCtrlOut.h"

namespace ausb::device {

void StallCtrlOut::start(const SetupPacket &packet) {
  error();
}

void StallCtrlOut::out_data_received(uint32_t bytes_received) {
  // We never call start_read(), so we don't expect to ever receive data
}

void StallCtrlOut::xfer_failed(XferFailReason reason) {}

} // namespace ausb::device
