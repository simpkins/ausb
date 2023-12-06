// Copyright (c) 2023, Adam Simpkins
#include "ausb/dev/ctrl/GetDevDescriptorModifyEP0.h"

#include "ausb/SetupPacket.h"
#include "ausb/log.h"

#include <cstring>

namespace ausb::device {

GetDevDescriptorModifyEP0::GetDevDescriptorModifyEP0(ControlEndpoint *ep,
                                                     buf_view buf,
                                                     uint8_t correct_ep0_mps)
    : CtrlInXfer(ep) {
  assert(desc_.data().size() == buf.size());
  memcpy(desc_.data().data(), buf.data(), desc_.data().size());
  desc_.set_max_pkt_size0(correct_ep0_mps);
}

void GetDevDescriptorModifyEP0::start(const SetupPacket &packet) {
  // It's unexpected if we try to respond with more data than was requested.
  // This could be caused by a badly behaving host, but it might indicate a
  // bug in our device code somewhere.
  if (desc_.data().size() > packet.length) {
    AUSB_LOGW("host requested partial device descriptor (%u bytes)",
              packet.length);
    send_full(desc_.data().data(), packet.length);
  } else {
    send_full(desc_.data().data(), desc_.data().size());
  }
}

void GetDevDescriptorModifyEP0::xfer_acked() {
  AUSB_LOGV("GET_DESCRIPTOR for modified device descriptor acked");
}

void GetDevDescriptorModifyEP0::xfer_failed(XferFailReason reason) {
  AUSB_LOGW("GET_DESCRIPTOR modified EP0 MPS xfer failed: reason=%d",
            static_cast<int>(reason));
}

} // namespace ausb::device
