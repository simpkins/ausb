// Copyright (c) 2023, Adam Simpkins
#include "ausb/device/StdControlHandler.h"

#include "ausb/SetupPacket.h"
#include "ausb/desc/DeviceDescriptor.h"
#include "ausb/device/CtrlInXfer.h"
#include "ausb/device/CtrlOutXfer.h"
#include "ausb/device/EndpointManager.h"
#include "ausb/device/Interface.h"
#include "ausb/device/ctrl/AckEmptyCtrlOut.h"
#include "ausb/device/ctrl/GetDevDescriptorModifyEP0.h"
#include "ausb/device/ctrl/GetStaticDescriptor.h"
#include "ausb/device/ctrl/SendData.h"
#include "ausb/device/ctrl/SetAddress.h"
#include "ausb/device/ctrl/StallCtrlIn.h"
#include "ausb/device/ctrl/StallCtrlOut.h"
#include "ausb/log.h"

namespace ausb::device {

void StdControlHandler::on_reset(XferFailReason reason) {
  callback_->on_reset(reason);
}

void StdControlHandler::on_enum_done(UsbSpeed speed,
                                     uint8_t ep0_max_packet_size) {
  ep0_max_packet_size_ = ep0_max_packet_size;
  callback_->on_bus_enumerated(speed);
}

void StdControlHandler::on_suspend() {
  callback_->on_suspend();
}

void StdControlHandler::on_resume() {
  callback_->on_resume();
}

CtrlOutXfer *StdControlHandler::process_out_setup(MessagePipe *pipe,
                                                  const SetupPacket &packet) {
  if (packet.request_type ==
      SetupPacket::make_request_type(
          Direction::Out, SetupRecipient::Device, SetupReqType::Standard)) {
    return process_std_device_out(pipe, packet);
  }

  const auto recip = packet.get_recipient();
  if (recip == SetupRecipient::Interface) {
    const uint8_t interface_num = (packet.index & 0xff);
    auto *const intf = pipe->manager()->get_interface(interface_num);
    if (!intf) {
      AUSB_LOGW("received SETUP OUT request for unknown interface %d",
                interface_num);
      return pipe->new_out_handler<StallCtrlOut>(pipe);
    }

    return intf->process_out_setup(pipe, packet);
  } else if (recip == SetupRecipient::Endpoint) {
    const uint8_t endpoint_addr = packet.index & 0xff;
    // TODO
    AUSB_LOGE("TODO: process SETUP OUT request for endpoint 0x%02x",
              endpoint_addr);
    return nullptr;
  }

  // We do not process any other request types:
  // - We only handle standard requests to the device
  // - We do not handle any vendor-specific requests.
  //   Users that wish to support vendor-specific requests should provide their
  //   own EndpointZeroCallback.  Their callback can delegate any requests they
  //   do not explicitly handle to a StdControlHandler in order to still get
  //   normal processing of all standard requests.
  AUSB_LOGW("rejecting unsupported SETUP OUT request 0x%02x 0x%02x",
            packet.request_type,
            packet.request);
  return pipe->new_out_handler<StallCtrlOut>(pipe);
}

CtrlInXfer *StdControlHandler::process_in_setup(MessagePipe *pipe,
                                                const SetupPacket &packet) {
  if (packet.request_type ==
      SetupPacket::make_request_type(
          Direction::In, SetupRecipient::Device, SetupReqType::Standard)) {
    return process_std_device_in(pipe, packet);
  }

  const auto recip = packet.get_recipient();
  if (recip == SetupRecipient::Interface) {
    const uint8_t interface_num = (packet.index & 0xff);
    auto *const intf = pipe->manager()->get_interface(interface_num);
    if (!intf) {
      AUSB_LOGW("received SETUP IN request for unknown interface %d",
                interface_num);
      return pipe->new_in_handler<StallCtrlIn>(pipe);
    }

    return intf->process_in_setup(pipe, packet);
  } else if (recip == SetupRecipient::Endpoint) {
    const uint8_t endpoint_addr = packet.index & 0xff;
    // TODO
    AUSB_LOGE("TODO: process SETUP IN request for endpoint 0x%02x",
              endpoint_addr);
    return nullptr;
  }

  // As above for OUT requests, we do not support any other request types
  AUSB_LOGW("rejecting unsupported SETUP IN request 0x%02x 0x%02x",
            packet.request_type,
            packet.request);
  return pipe->new_in_handler<StallCtrlIn>(pipe);
}

CtrlOutXfer *
StdControlHandler::process_std_device_out(MessagePipe *pipe,
                                          const SetupPacket &packet) {
  const auto std_req_type = packet.get_std_request();
  if (std_req_type == StdRequestType::SetAddress) {
    callback_->on_address_set(packet.value);
    return pipe->new_out_handler<SetAddress>(pipe);
  } else if (std_req_type == StdRequestType::SetConfiguration) {
    return process_set_configuration(pipe, packet);
  } else if (std_req_type == StdRequestType::SetFeature) {
    // TODO
    AUSB_LOGE("TODO: handle SET_FEATURE");
    return nullptr;
  } else if (std_req_type == StdRequestType::ClearFeature) {
    // TODO
    AUSB_LOGE("TODO: handle CLEAR_FEATURE");
    return nullptr;
  } else if (std_req_type == StdRequestType::SetDescriptor) {
    // We don't support SET_DESCRIPTOR calls targeted at the device.
    // Some device classes might support this for some interfaces (e.g., HID
    // devices might allow SET_DESCRIPTOR calls for HID descriptors), but we
    // don't allow changing any of the device-level descriptors.
    AUSB_LOGW("rejecting SET_DESCRIPTOR call for device-level descriptor "
              "0x%04x 0x%04x",
              packet.value,
              packet.index);
    return pipe->new_out_handler<StallCtrlOut>(pipe);
  }

  AUSB_LOGW("unknown standard device OUT request %u",
            static_cast<unsigned int>(std_req_type));
  return nullptr;
}

CtrlInXfer *
StdControlHandler::process_std_device_in(MessagePipe *pipe,
                                         const SetupPacket &packet) {
  const auto std_req_type = packet.get_std_request();
  if (std_req_type == StdRequestType::GetDescriptor) {
    return process_get_descriptor(pipe, packet);
  } else if (std_req_type == StdRequestType::GetStatus) {
    // TODO: return remote wakeup and self-powered status
    AUSB_LOGE("TODO: handle GET_STATUS");
    return nullptr;
  } else if (std_req_type == StdRequestType::GetConfiguration) {
    return pipe->new_in_handler<SendU8>(pipe, pipe->manager()->config_id());
  }

  AUSB_LOGW("unknown standard device IN request %u",
            static_cast<unsigned int>(std_req_type));
  return nullptr;
}

CtrlOutXfer *
StdControlHandler::process_set_configuration(MessagePipe *pipe,
                                             const SetupPacket &packet) {
  if (packet.length > 0) {
    AUSB_LOGW("SET_CONFIGURATION request with non-zero length %d",
              packet.length);
    return pipe->new_out_handler<StallCtrlOut>(pipe);
  }
  const uint8_t config_id = packet.value;
  AUSB_LOGI("SET_CONFIGURATION: %u", config_id);

  const auto state = pipe->manager()->state();
  if (state != DeviceState::Address && state != DeviceState::Configured) {
    return pipe->new_out_handler<StallCtrlOut>(pipe);
  }

  if (config_id == 0) {
    pipe->manager()->unconfigure();
  } else if (!callback_->set_configuration(config_id)) {
    AUSB_LOGW("rejected SET_CONFIGURATION %u request", config_id);
    // Note: if a SetConfiguration request is received with an invalid config
    // ID, the USB spec specifies that we should reply with an error, but does
    // not really specify what state we should be in afterwards if we were
    // already in a configured state before.
    return pipe->new_out_handler<StallCtrlOut>(pipe);
  }

  return pipe->new_out_handler<AckEmptyCtrlOut>(pipe);
}

CtrlInXfer *
StdControlHandler::process_get_descriptor(MessagePipe *pipe,
                                          const SetupPacket &packet) {
  const auto desc = callback_->get_descriptor(packet.value, packet.index);
  if (!desc) {
    // The host requested a descriptor that does not exist.
    // This is expected in many cases.  e.g., devices that do not support
    // high-speed operation are intentionally supposed to fail requests for the
    // DeviceQualifier descriptor.
    AUSB_LOGI(
        "GET_DESCRIPTOR request for non-existent descriptor 0x%04x 0x%04x",
        packet.value,
        packet.index);
    return pipe->new_in_handler<StallCtrlIn>(pipe);
  }

  AUSB_LOGI(
      "GET_DESCRIPTOR request for 0x%04x 0x%04x", packet.value, packet.index);

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
      AUSB_LOGV("GET_DESCRIPTOR explicitly modifying device descriptor to set "
                "correct EP0 max packet size (%u -> %u)",
                (*desc)[7],
                ep0_max_packet_size_);
      return pipe->new_in_handler<GetDevDescriptorModifyEP0>(
          pipe, *desc, ep0_max_packet_size_);
    }
  }

  return pipe->new_in_handler<GetStaticDescriptor>(
      pipe, desc->data(), desc->size());
}

} // namespace ausb::device
