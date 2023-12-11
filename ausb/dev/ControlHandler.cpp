// Copyright (c) 2023, Adam Simpkins
#include "ausb/dev/ControlHandler.h"

#include "ausb/SetupPacket.h"
#include "ausb/desc/DeviceDescriptor.h"
#include "ausb/dev/CtrlInXfer.h"
#include "ausb/dev/CtrlOutXfer.h"
#include "ausb/dev/EndpointManager.h"
#include "ausb/dev/Interface.h"
#include "ausb/dev/ctrl/AckEmptyCtrlOut.h"
#include "ausb/dev/ctrl/GetDevDescriptorModifyEP0.h"
#include "ausb/dev/ctrl/GetStaticDescriptor.h"
#include "ausb/dev/ctrl/SetAddress.h"
#include "ausb/dev/ctrl/StallCtrlIn.h"
#include "ausb/dev/ctrl/StallCtrlOut.h"
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

  const auto recip = packet.get_recipient();
  if (recip == SetupRecipient::Interface) {
    const uint8_t interface_num = (packet.index & 0xff);
    auto *const intf = endpoint_->manager()->get_interface(interface_num);
    if (!intf) {
      AUSB_LOGW("received SETUP OUT request for unknown interface %d",
                interface_num);
      return nullptr;
    }

    return intf->process_out_setup(endpoint_, packet);
  } else if (recip == SetupRecipient::Endpoint) {
    const uint8_t endpoint_addr = packet.index & 0xff;
    // TODO
    (void)endpoint_addr;
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

  const auto recip = packet.get_recipient();
  if (recip == SetupRecipient::Interface) {
    const uint8_t interface_num = (packet.index & 0xff);
    auto *const intf = endpoint_->manager()->get_interface(interface_num);
    if (!intf) {
      AUSB_LOGW("received SETUP IN request for unknown interface %d",
                interface_num);
      return nullptr;
    }

    return intf->process_in_setup(endpoint_, packet);
  } else if (recip == SetupRecipient::Endpoint) {
    const uint8_t endpoint_addr = packet.index & 0xff;
    // TODO
    (void)endpoint_addr;
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
  } else if (std_req_type == StdRequestType::SetConfiguration) {
    return process_set_configuration(packet);
  } else if (std_req_type == StdRequestType::SetFeature) {
    // TODO
    AUSB_LOGE("TODO: handle SET_FEATURE");
    return nullptr;
  } else if (std_req_type == StdRequestType::ClearFeature) {
    // TODO
    AUSB_LOGE("TODO: handle CLEAR_FEATURE");
    return nullptr;
  } else if (std_req_type == StdRequestType::SetDescriptor) {
    // TODO
    AUSB_LOGE("TODO: handle SET_DESCRIPTOR");
    return nullptr;
  }

  AUSB_LOGW("unknown standard device OUT request %u",
            static_cast<unsigned int>(std_req_type));
  return nullptr;
}

std::unique_ptr<CtrlInXfer>
ControlHandler::process_std_device_in(const SetupPacket &packet) {
  const auto std_req_type = packet.get_std_request();
  if (std_req_type == StdRequestType::GetDescriptor) {
    return process_get_descriptor(packet);
  } else if (std_req_type == StdRequestType::GetStatus) {
    // TODO: return remote wakeup and self-powered status
    AUSB_LOGE("TODO: handle GET_STATUS");
    return nullptr;
  } else if (std_req_type == StdRequestType::GetConfiguration) {
    // TODO: return current config ID
    AUSB_LOGE("TODO: handle GET_CONFIGURATION");
    return nullptr;
  }

  AUSB_LOGW("unknown standard device IN request %u",
            static_cast<unsigned int>(std_req_type));
  return nullptr;
}

std::unique_ptr<CtrlOutXfer>
ControlHandler::process_set_configuration(const SetupPacket &packet) {
  if (packet.length > 0) {
    AUSB_LOGW("SET_CONFIGURATION request with non-zero length %d",
              packet.length);
    return make_unique<StallCtrlOut>(endpoint_);
  }
  const uint8_t config_id = packet.value;
  AUSB_LOGI("SET_CONFIGURATION: %u", config_id);

  const auto state = endpoint_->manager()->state();
  if (state != DeviceState::Address && state != DeviceState::Configured) {
    return make_unique<StallCtrlOut>(endpoint_);
  }

  // TODO: should we perhaps handle the config_id == 0 case specially, rather
  // than requiring callback_ to handle this on their own?
  if (!callback_->set_configuration(config_id)) {
    AUSB_LOGW("rejected SET_CONFIGURATION %u request", config_id);
    // Note: if a SetConfiguration request is received with an invalid config
    // ID, the USB spec specifies that we should reply with an error, but does
    // not really specify what state we should be in afterwards if we were
    // already in a configured state before.
    return make_unique<StallCtrlOut>(endpoint_);
  }

  return make_unique<AckEmptyCtrlOut>(endpoint_);
}

std::unique_ptr<CtrlInXfer>
ControlHandler::process_get_descriptor(const SetupPacket &packet) {
  const auto desc = callback_->get_descriptor(packet.value, packet.index);
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
  if (packet.value == desc_setup_value(DescriptorType::Device) &&
      packet.index == 0) {
    DeviceDescriptorParser dd(*desc);
    if (dd.valid() && dd.ep0_max_pkt_size() != ep0_max_packet_size_) {
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
