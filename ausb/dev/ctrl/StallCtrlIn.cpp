// Copyright (c) 2023, Adam Simpkins
#include "ausb/dev/ctrl/StallCtrlIn.h"

#include "ausb/log.h"

namespace ausb::device {

void StallCtrlIn::start(const SetupPacket &packet) {
  error();
}

void StallCtrlIn::xfer_acked() {
  // We don't ever expect xfer_acked() to be called
  AUSB_LOGE("xfer_acked() called for StallCtrlIn");
}

void StallCtrlIn::xfer_failed(XferFailReason reason) {
  // We don't ever expect xfer_failed() to be called after we explicitly call
  // error
  AUSB_LOGE("xfer_failed() called for StallCtrlIn");
}

} // namespace ausb::device
