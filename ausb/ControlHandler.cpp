// Copyright (c) 2023, Adam Simpkins
#include "ausb/ControlHandler.h"

#include "ausb/CtrlInXfer.h"
#include "ausb/CtrlOutXfer.h"
#include "ausb/SetupPacket.h"
#include "ausb/desc/DeviceDescriptor.h"
#include "ausb/dev/ctrl/GetStaticDescriptor.h"
#include "ausb/dev/ctrl/GetDevDescriptorModifyEP0.h"
#include "ausb/dev/ctrl/SetAddress.h"
#include "ausb/dev/ctrl/StallCtrlIn.h"
#include "ausb/log.h"

using std::make_unique;

namespace ausb::device {

void ControlHandler::on_enum_done(uint8_t max_packet_size) {
  ep0_max_packet_size_ = max_packet_size;
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
    return make_unique<SetAddress>(endpoint_);
  }

  AUSB_LOGE("TODO: unhandled OUT std device setup packet");
  return nullptr;
}

std::unique_ptr<CtrlInXfer>
ControlHandler::process_std_device_in(const SetupPacket &packet) {
  const auto std_req_type = packet.get_std_request();
  if (std_req_type == StdRequestType::GetDescriptor) {
    return process_get_descriptor(packet);
  }

  // TODO
  AUSB_LOGW("TODO: unhandled IN std device setup packet");
  return nullptr;
}

std::unique_ptr<CtrlInXfer>
ControlHandler::process_get_descriptor(const SetupPacket &packet) {
  const auto desc =
      descriptors_->get_descriptor_for_setup(packet.value, packet.index);
  if (!desc) {
    // The host requested a descriptor that does not exist.
    // This is expected in many cases.  e.g., devices that do not support
    // high-speed operation are intentionally supposed to fail requests for the
    // DeviceQualifier descriptor.
    AUSB_LOGI(
        "GET_DESCRIPTOR request for non-existent descriptor 0x%04x 0x%04x",
        packet.value, packet.index);
    return make_unique<StallCtrlIn>(endpoint_);
  }

  AUSB_LOGI("GET_DESCRIPTOR request for 0x%04x 0x%04x", packet.value,
            packet.index);

  // We do special handling for the device descriptor.
  // If the max endpoint size in the device descriptor does not match the
  // currently configured max packet size, we create a temporary copy of the
  // descriptor so we can modify it to contain the correct max packet size.
  //
  // (Note that it should hopefully be pretty rare that we should have to do
  // this.  In most cases users should define their device descriptor with a
  // max packet size of 64 and this should be the configured max packet size in
  // most cases.  Only if low speed was negotiated should we have to use a max
  // packet size of 8, and I expect there aren't many low speed hosts/hubs in
  // use any more.)
  if (packet.value == 0x0100 && packet.index == 0) {
    if (desc->size() == DeviceDescriptor::kSize &&
        (*desc)[7] != ep0_max_packet_size_) {
      AUSB_LOGI("GET_DESCRIPTOR explicitly modifying device descriptor to set "
                "correct EP0 max packet size (%u -> %u)",
                (*desc)[7], ep0_max_packet_size_);
      return make_unique<GetDevDescriptorModifyEP0>(endpoint_, *desc,
                                                    ep0_max_packet_size_);
    }
  }

  return make_unique<GetStaticDescriptor>(endpoint_, desc->data(),
                                          desc->size());
}

} // namespace ausb::device
