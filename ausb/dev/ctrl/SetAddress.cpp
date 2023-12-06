// Copyright (c) 2023, Adam Simpkins
#include "ausb/dev/ctrl/SetAddress.h"

#include "ausb/SetupPacket.h"
#include "ausb/UsbDevice.h"
#include "ausb/log.h"

namespace ausb::device {

void SetAddress::start(const SetupPacket &packet) {
  if (packet.length > 0) {
    error();
  }
  const uint8_t address = packet.value;
  AUSB_LOGI("SET_ADDRESS: %u", packet.value);
  UsbDevice *usb = endpoint()->usb();
  usb->set_address(address);
  ack();
}

void SetAddress::out_data_received(uint32_t bytes_received) {
  // We never call start_read(), so we don't expect to ever receive data
}

void SetAddress::xfer_failed(XferFailReason reason) {}

} // namespace ausb::device
