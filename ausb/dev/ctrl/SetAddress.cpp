// Copyright (c) 2023, Adam Simpkins
#include "ausb/dev/ctrl/SetAddress.h"

#include "ausb/SetupPacket.h"
#include "ausb/dev/EndpointManager.h"
#include "ausb/log.h"

namespace ausb::device {

void SetAddress::start(const SetupPacket &packet) {
  if (packet.length > 0) {
    error();
  }
  const uint8_t address_ = packet.value;
  AUSB_LOGI("SET_ADDRESS: %u", packet.value);

  // The SET_ADDRESS is somewhat unique compared to all other SETUP requests:
  // for all other SETUP requests, the device must complete request processing
  // before acknowledging the transfer.  However, for SET_ADDRESS the ack needs
  // to be sent with the original address, and so the address itself should not
  // be changed until after the ack has completed.  We do still make a call to
  // the underlying hardware here just in case some hardware implementations
  // need to be informed earlier during the transfer.
  endpoint()->manager()->set_address_early(address_);

  ack();
}

void SetAddress::out_data_received(uint32_t bytes_received) {
  // We never call start_read(), so we don't expect to ever receive data
}

void SetAddress::xfer_failed(XferFailReason reason) {}

void SetAddress::ack_complete() {
  // Apply the address once the status stage of this transfer has finished.
  endpoint()->manager()->set_address(address_);
}

} // namespace ausb::device
