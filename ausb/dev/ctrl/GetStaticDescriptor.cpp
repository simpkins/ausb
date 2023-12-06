// Copyright (c) 2023, Adam Simpkins
#include "ausb/dev/ctrl/GetStaticDescriptor.h"

#include "ausb/SetupPacket.h"
#include "ausb/log.h"

namespace ausb::device {

void GetStaticDescriptor::start(const SetupPacket &packet) {
  // It's unexpected if we try to respond with more data than was requested.
  // This could be caused by a badly behaving host, but it might indicate a
  // bug in our device code somewhere.
  if (size_ > packet.length) {
    AUSB_LOGW("data provided for GET_DESCRIPTOR response is longer than host "
              "requested.  descriptor will be truncated.");
    send_full(data_, packet.length);
  } else {
    send_full(data_, size_);
  }
}

void GetStaticDescriptor::xfer_acked() {
  AUSB_LOGV("GET_DESCRIPTOR xfer acked");
}

void GetStaticDescriptor::xfer_failed(XferFailReason reason)  {
  AUSB_LOGW("GET_DESCRIPTOR xfer failed: reason=%d", static_cast<int>(reason));
}

} // namespace ausb::device
