// Copyright (c) 2023, Adam Simpkins
#include "ausb/dev/ctrl/GetConfiguration.h"

#include "ausb/SetupPacket.h"
#include "ausb/log.h"

namespace ausb::device {

void GetConfiguration::start(const SetupPacket &packet) {
  if (packet.length == 0) {
    AUSB_LOGW("received GET_CONFIGURATION request with no room for reply data");
    error();
    return;
  }
  send_full(&config_id_, sizeof(config_id_));
}

void GetConfiguration::xfer_acked() {}

void GetConfiguration::xfer_failed(XferFailReason reason) {
  AUSB_LOGW("GET_CONFIGURATION xfer failed: reason=%d",
            static_cast<int>(reason));
}

} // namespace ausb::device
