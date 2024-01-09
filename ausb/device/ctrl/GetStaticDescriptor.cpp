// Copyright (c) 2023, Adam Simpkins
#include "ausb/device/ctrl/GetStaticDescriptor.h"

#include "ausb/SetupPacket.h"
#include "ausb/log.h"

namespace ausb::device {

void GetStaticDescriptor::start(const SetupPacket &packet) {
  // If the host requested less than the full descriptor length, just send
  // the first portion.  This is common for the config descriptor: the host
  // does not know the full length, so it requests just the basic descriptor
  // header first, which includes the total length.  Afterwards it can then
  // request the full descriptor.
  send_full(data_, std::min(packet.length, static_cast<uint16_t>(size_)));
}

void GetStaticDescriptor::xfer_acked() {
  AUSB_LOGV("GET_DESCRIPTOR xfer acked");
}

void GetStaticDescriptor::xfer_failed(XferFailReason reason) {
  AUSB_LOGW("GET_DESCRIPTOR xfer failed: reason=%d", static_cast<int>(reason));
}

} // namespace ausb::device
