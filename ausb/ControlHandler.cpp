// Copyright (c) 2023, Adam Simpkins
#include "ausb/ControlHandler.h"

#include "ausb/log.h"
#include "ausb/CtrlInXfer.h"
#include "ausb/CtrlOutXfer.h"
#include "ausb/SetupPacket.h"
#include "ausb/dev/ctrl/GetStaticDescriptor.h"
#include "ausb/dev/ctrl/SetAddress.h"

namespace ausb::device {

void ControlHandler::on_enum_done(uint8_t max_packet_size) {
  // Update the device descriptor with the selected EP0 max packet size
  dev_descriptor_.set_max_pkt_size0(max_packet_size);
}

std::unique_ptr<CtrlOutXfer>
ControlHandler::process_out_setup(const SetupPacket &packet) {
  if (packet.request_type ==
      SetupPacket::make_request_type(Direction::Out, SetupRecipient::Device,
                                     SetupReqType::Standard)) {
    return process_std_device_out(packet);
  }

  // TODO:
  AUSB_LOGE("TODO: process OUT setup packet");
  return nullptr;
}

std::unique_ptr<CtrlInXfer>
ControlHandler::process_in_setup(const SetupPacket &packet) {
  if (packet.request_type ==
      SetupPacket::make_request_type(Direction::In, SetupRecipient::Device,
                                     SetupReqType::Standard)) {
    return process_std_device_in(packet);
  }

  // TODO:
  AUSB_LOGE("TODO: handle control IN transfer");
  return nullptr;
}

std::unique_ptr<CtrlOutXfer>
ControlHandler::process_std_device_out(const SetupPacket &packet) {
  const auto std_req_type = packet.get_std_request();
  if (std_req_type == StdRequestType::SetAddress) {
    return std::make_unique<SetAddress>(endpoint_);
  }

  AUSB_LOGE("TODO: unhandled OUT std device setup packet");
  return nullptr;
}

std::unique_ptr<CtrlInXfer>
ControlHandler::process_std_device_in(const SetupPacket &packet) {
  const auto std_req_type = packet.get_std_request();
  if (std_req_type == StdRequestType::GetDescriptor) {
    if (packet.value == 0x0100 && packet.index == 0) {
      AUSB_LOGI("process GET_DESCRIPTOR request for device descriptor");
      return std::make_unique<GetStaticDescriptor>(endpoint_,
                                                   dev_descriptor_.data());
    } else {
      AUSB_LOGW("TODO: process GET_DESCRIPTOR request");
    }
  }

  // TODO
  AUSB_LOGW("TODO: unhandled IN std device setup packet");
  return nullptr;
}

} // namespace ausb::device
