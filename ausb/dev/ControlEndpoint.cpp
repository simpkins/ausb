// Copyright (c) 2023, Adam Simpkins
#include "ausb/dev/ControlEndpoint.h"

#include "ausb/SetupPacket.h"
#include "ausb/dev/EndpointManager.h"
#include "ausb/log.h"

#include <cassert>

namespace ausb::device {

ControlEndpoint::~ControlEndpoint() = default;

void ControlEndpoint::on_reset(XferFailReason reason) {
  pipe_.on_unconfigured(reason);
  // We don't stall the endpoint here.  The reset itself should flush
  // all transfers from the hardware and disconnect the bus.

  get_callback()->on_reset(reason);
}

void ControlEndpoint::on_enum_done(uint8_t max_packet_size) {
  get_callback()->on_enum_done(max_packet_size);
}

void ControlEndpoint::on_suspend() {
  get_callback()->on_suspend();
}

void ControlEndpoint::on_resume() {
  get_callback()->on_suspend();
}

} // namespace ausb::device
