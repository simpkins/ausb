// Copyright (c) 2023, Adam Simpkins
#include "ausb/dev/ctrl/AckEmptyCtrlOut.h"

#include "ausb/SetupPacket.h"

namespace ausb::device {

void AckEmptyCtrlOut::start(const SetupPacket &packet) {
  if (packet.length != 0) {
    error();
  }
  ack();
}

void AckEmptyCtrlOut::out_data_received(uint32_t bytes_received) {
  // We never call start_read(), so we don't expect to ever receive data
}

void AckEmptyCtrlOut::xfer_failed(XferFailReason reason) {}

} // namespace ausb::device
