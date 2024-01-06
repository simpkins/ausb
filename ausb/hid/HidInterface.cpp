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
      AUSB_LOGW("unsupported SET_DESCRIPTOR request for HID descriptor 0x%04x",
                packet.value);
      return pipe->new_out_handler<StallCtrlOut>(pipe);
    }
  } else if (req_type == SetupReqType::Class) {
    const auto hid_req_type = static_cast<HidRequest>(packet.request);
    if (hid_req_type == HidRequest::SetIdle) {
      const uint8_t duration_4ms = (packet.value >> 8) & 0xff;
      const uint8_t report_id = packet.value & 0xff;
      AUSB_LOGI(
          "HID SET_IDLE: report_id=%u duration=%u", report_id, duration_4ms);
      auto report_queue = report_map_->get_report_queue(report_id);
      if (!report_queue) {
        AUSB_LOGW("received SET_IDLE request for unknown HID report %d",
                  report_id);
        return pipe->new_out_handler<StallCtrlOut>(pipe);
      }
      report_queue->set_idle_4ms(duration_4ms);
      return pipe->new_out_handler<AckEmptyCtrlOut>(pipe);
    } else if (hid_req_type == HidRequest::SetReport) {
      AUSB_LOGE("TODO: HID SET_REPORT");
      // TODO
      return pipe->new_out_handler<HidSetReport>(pipe, this);
    } else if (hid_req_type == HidRequest::SetProtocol) {
      AUSB_LOGE("TODO: HID SET_PROTOCOL");
      // TODO: inform a callback and let it decide how to respond
      return pipe->new_out_handler<AckEmptyCtrlOut>(pipe);
    }
  }

  AUSB_LOGW("unhandled control OUT request (0x%#02x 0x%#02x) to HID interface",
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
      AUSB_LOGI("HID GET_DESCRIPTOR: value=%u", packet.value);
      // In practice it seems like there is only ever a single HID class
      // descriptor.  The HID specification indicates that there should be "at
      // least one" report descriptor, but there doesn't seem to be a use for
      // more than 1, and host implementations appear to generally expect
      // exactly one report descriptor.
      //
      // The HID specification also defines physical descriptors, but
      // generally discourages implementing them.  I haven't come across
      // anything that seems to use them in practice.  The Linux USB HID code
      // never appears to request HID physical descriptors.
      if (packet.value == desc_setup_value(DescriptorType::HidReport, 0)) {
        return pipe->new_in_handler<GetStaticDescriptor>(
            pipe, report_descriptor_, report_descriptor_size_);
      } else {
        AUSB_LOGV("GET_DESCRIPTOR request for non-existent HID class "
                  "descriptor 0x%04x",
                  packet.value);
        return pipe->new_in_handler<StallCtrlIn>(pipe);
      }
    }
  } else if (req_type == SetupReqType::Class) {
    const auto hid_req_type = static_cast<HidRequest>(packet.request);
    if (hid_req_type == HidRequest::GetReport) {
      AUSB_LOGE("TODO: HID GET_REPORT");
      // TODO
    } else if (hid_req_type == HidRequest::GetIdle) {
      AUSB_LOGE("TODO: HID GET_IDLE");
      // TODO
    } else if (hid_req_type == HidRequest::GetProtocol) {
      AUSB_LOGE("TODO: HID GET_PROTOCOL");
      // TODO
    }
  }

  AUSB_LOGW("unhandled control IN request (0x%#02x 0x%#02x) to HID interface",
            packet.request_type,
            packet.request);
  return nullptr;
}

} // namespace ausb::hid
