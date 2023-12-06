// Copyright (c) 2023, Adam Simpkins
#include "ausb/ControlHandler.h"

#include "ausb/log.h"
#include "ausb/CtrlInXfer.h"
#include "ausb/CtrlOutXfer.h"

namespace ausb::device {

void ControlHandler::on_enum_done(uint8_t max_packet_size) {
  // TODO: update the max packet size in the device descriptor
}

std::unique_ptr<CtrlOutXfer>
ControlHandler::process_out_setup(const SetupPacket &packet) {
  // TODO:
  AUSB_LOGE("TODO: handle control OUT transfer");
  return nullptr;
}

std::unique_ptr<CtrlInXfer>
ControlHandler::process_in_setup(const SetupPacket &packet) {
  // TODO:
  AUSB_LOGE("TODO: handle control IN transfer");
  return nullptr;
}

} // namespace ausb::device
