// Copyright (c) 2023, Adam Simpkins
#include "ausb/device/ctrl/AckEmptyCtrlOut.h"

#include "ausb/SetupPacket.h"
#include "ausb/log.h"

namespace ausb::device {

void AckEmptyCtrlOut::start(const SetupPacket &packet) {
  if (packet.length != 0) {
    AUSB_LOGE(
        "received OUT SETUP packet with unexpected non-zero length %" PRIu16
        " request_type=%#" PRIx16 " request=%#" PRIx16,
        packet.length,
        packet.request_type,
        packet.request);
    error();
  }
  ack();
}

void AckEmptyCtrlOut::out_data_received(uint32_t bytes_received) {
  // We never call start_read(), so we don't expect to ever receive data
}

void AckEmptyCtrlOut::xfer_failed(XferFailReason reason) {}

} // namespace ausb::device
