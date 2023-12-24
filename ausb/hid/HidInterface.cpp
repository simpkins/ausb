// Copyright (c) 2023, Adam Simpkins
#include "ausb/hid/HidInterface.h"

#include "ausb/SetupPacket.h"
#include "ausb/desc/types.h"
#include "ausb/dev/CtrlInXfer.h"
#include "ausb/dev/CtrlOutXfer.h"
#include "ausb/dev/MessagePipe.h"
#include "ausb/dev/ctrl/AckEmptyCtrlOut.h"
#include "ausb/dev/ctrl/GetStaticDescriptor.h"
#include "ausb/dev/ctrl/StallCtrlIn.h"
#include "ausb/dev/ctrl/StallCtrlOut.h"
#include "ausb/hid/HidSetReport.h"
#include "ausb/hid/types.h"

#include "ausb/log.h"

using namespace ausb::device;

namespace ausb::hid {

CtrlOutXfer *HidInterface::process_out_setup(MessagePipe *pipe,
                                             const SetupPacket &packet) {
  const auto req_type = packet.get_request_type();
  if (req_type == SetupReqType::Standard) {
    const auto std_req_type = packet.get_std_request();
    if (std_req_type == StdRequestType::SetDescriptor) {
      // We don't currently support SET_DESCRIPTOR
      AUSB_LOGI("unsupported SET_DESCRIPTOR request for HID descriptor 0x%04x",
                packet.value);
      return nullptr;
    }
  } else if (req_type == SetupReqType::Class) {
    const auto hid_req_type = static_cast<HidRequest>(packet.request);
    if (hid_req_type == HidRequest::SetIdle) {
      const uint8_t duration = (packet.value >> 8) & 0xff;
      const uint8_t report_id = packet.value & 0xff;
      auto report_queue = report_map_->get_report_queue(report_id);
      if (!report_queue) {
        AUSB_LOGW("received SET_IDLE request for unknown HID report %d",
                  report_id);
        return pipe->new_out_handler<StallCtrlOut>(pipe);
      }
      // TODO: actually record and act on the idle setting
      // report_queue->set_idle_4ms(duration);
      (void)duration;
      return pipe->new_out_handler<AckEmptyCtrlOut>(pipe);
    } else if (hid_req_type == HidRequest::SetReport) {
      // TODO
      return pipe->new_out_handler<HidSetReport>(pipe, this);
    } else if (hid_req_type == HidRequest::SetProtocol) {
      // TODO: inform a callback and let it decide how to respond
      return pipe->new_out_handler<AckEmptyCtrlOut>(pipe);
    }

    AUSB_LOGW("unhandled HID request %u to HID keyboard interface",
              packet.request);
    return nullptr;
  }

  AUSB_LOGW(
      "unhandled control request (0x%#02x 0x%#02x) to HID keyboard interface",
      packet.request_type,
      packet.request);
  return nullptr;
}

CtrlInXfer *HidInterface::process_in_setup(MessagePipe *pipe,
                                           const SetupPacket &packet) {
  const auto req_type = packet.get_request_type();
  if (req_type == SetupReqType::Standard) {
    const auto std_req_type = packet.get_std_request();
    if (std_req_type == StdRequestType::GetDescriptor) {
      if (packet.value == desc_setup_value(DescriptorType::HidReport, 0)) {
        return pipe->new_in_handler<GetStaticDescriptor>(
            pipe, report_descriptor_, report_descriptor_size_);
      } else {
        // Hosts can request physical descriptors.
        // We currently do not support returning physical descriptors.
        AUSB_LOGV("GET_DESCRIPTOR request for non-existent HID class "
                  "descriptor 0x%04x",
                  packet.value);
        return pipe->new_in_handler<StallCtrlIn>(pipe);
      }
    }
  } else if (req_type == SetupReqType::Class) {
    const auto hid_req_type = static_cast<HidRequest>(packet.request);
    if (hid_req_type == HidRequest::GetReport) {
      // TODO
    } else if (hid_req_type == HidRequest::GetIdle) {
      // TODO
    } else if (hid_req_type == HidRequest::GetProtocol) {
      // TODO
    }
  }

  // TODO:
  AUSB_LOGE("TODO: implement HID Keyboard ctrl IN handling");
  return nullptr;
}

} // namespace ausb::hid
