// Copyright (c) 2023, Adam Simpkins
#include "ausb/device/EndpointZero.h"

#include "ausb/SetupPacket.h"
#include "ausb/device/EndpointManager.h"
#include "ausb/log.h"

#include <cassert>

namespace ausb::device {

EndpointZero::~EndpointZero() = default;

void EndpointZero::on_reset(XferFailReason reason) {
  pipe_.on_unconfigured(reason);
  // We don't stall the endpoint here.  The reset itself should flush
  // all transfers from the hardware and disconnect the bus.

  get_callback()->on_reset(reason);
}

void EndpointZero::on_enum_done(UsbSpeed speed, uint8_t ep0_max_packet_size) {
  get_callback()->on_enum_done(speed, ep0_max_packet_size);
}

void EndpointZero::on_suspend() {
  get_callback()->on_suspend();
}

void EndpointZero::on_resume() {
  get_callback()->on_suspend();
}

} // namespace ausb::device
