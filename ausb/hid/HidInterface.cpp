// Copyright (c) 2023, Adam Simpkins
#include "ausb/hid/HidInterface.h"

#include "ausb/SetupPacket.h"
#include "ausb/desc/types.h"
#include "ausb/dev/CtrlInXfer.h"
#include "ausb/dev/CtrlOutXfer.h"
#include "ausb/dev/MessagePipe.h"
#include "ausb/dev/ctrl/AckEmptyCtrlOut.h"
#include "ausb/dev/ctrl/GetStaticDescriptor.h"
#include "ausb/dev/ctrl/SendData.h"
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
      return set_idle(pipe, packet);
    } else if (hid_req_type == HidRequest::SetReport) {
      return set_report(pipe, packet);
    } else if (hid_req_type == HidRequest::SetProtocol) {
      return set_protocol(pipe, packet);
    }
  }

  AUSB_LOGW("unhandled control OUT request (0x%#02x 0x%#02x) to HID interface",
            packet.request_type,
            packet.request);
  return nullptr;
}

CtrlOutXfer *HidInterface::set_idle(MessagePipe *pipe,
                                    const SetupPacket &packet) {
  const uint8_t duration_4ms = (packet.value >> 8) & 0xff;
  const uint8_t report_id = packet.value & 0xff;
  AUSB_LOGI("HID SET_IDLE: report_id=%u duration=%u", report_id, duration_4ms);

  auto report_queue = get_report_queue(report_id);
  if (!report_queue) {
    AUSB_LOGW("received SET_IDLE request for unknown HID report %d", report_id);
    return pipe->new_out_handler<StallCtrlOut>(pipe);
  }
  report_queue->set_idle_4ms(duration_4ms);
  return pipe->new_out_handler<AckEmptyCtrlOut>(pipe);
}

CtrlOutXfer *HidInterface::set_report(MessagePipe *pipe,
                                    const SetupPacket &packet) {
  const uint8_t report_type_u8 = (packet.value >> 8) & 0xff;
  const uint8_t report_id = packet.value & 0xff;
  AUSB_LOGI(
      "HID SET_REPORT: report_type=%u report_id=%u", report_type_u8, report_id);
  const auto report_type = static_cast<HidReportType>(report_type_u8);

  AUSB_LOGE("TODO: HID SET_REPORT");
  // TODO
  (void)report_type;
  return pipe->new_out_handler<HidSetReport>(pipe, this);
}

CtrlOutXfer *HidInterface::set_protocol(MessagePipe *pipe,
                                    const SetupPacket &packet) {
  AUSB_LOGI("HID SET_PROTOCOL %" PRIu16, packet.value);
  if (packet.value == static_cast<uint16_t>(protocol_)) {
    return pipe->new_out_handler<AckEmptyCtrlOut>(pipe);
  }

  if (packet.value == 0) {
    if (boot_report_map_ == nullptr) {
      // This interface does not support the boot protocol
      return pipe->new_out_handler<StallCtrlOut>(pipe);
    }
    protocol_ = HidReportProtocol::Boot;
    in_endpoint_.change_protocol(boot_report_map_);
    return pipe->new_out_handler<AckEmptyCtrlOut>(pipe);
  } else if (packet.value == 1) {
    protocol_ = HidReportProtocol::Report;
    in_endpoint_.change_protocol(report_map_);
    return pipe->new_out_handler<AckEmptyCtrlOut>(pipe);
  } else {
    return pipe->new_out_handler<StallCtrlOut>(pipe);
  }
}

CtrlInXfer *HidInterface::process_in_setup(MessagePipe *pipe,
                                           const SetupPacket &packet) {
  const auto req_type = packet.get_request_type();
  if (req_type == SetupReqType::Standard) {
    const auto std_req_type = packet.get_std_request();
    if (std_req_type == StdRequestType::GetDescriptor) {
      return get_descriptor(pipe, packet);
    }
  } else if (req_type == SetupReqType::Class) {
    const auto hid_req_type = static_cast<HidRequest>(packet.request);
    if (hid_req_type == HidRequest::GetReport) {
      return get_report(pipe, packet);
    } else if (hid_req_type == HidRequest::GetIdle) {
      return get_idle(pipe, packet);
    } else if (hid_req_type == HidRequest::GetProtocol) {
      return get_protocol(pipe, packet);
    }
  }

  AUSB_LOGW("unhandled control IN request (0x%#02x 0x%#02x) to HID interface",
            packet.request_type,
            packet.request);
  return nullptr;
}

CtrlInXfer *HidInterface::get_descriptor(MessagePipe *pipe,
                                         const SetupPacket &packet) {
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

CtrlInXfer *HidInterface::get_report(MessagePipe *pipe,
                                         const SetupPacket &packet) {
  const uint8_t report_type_u8 = (packet.value >> 8) & 0xff;
  const uint8_t report_id = packet.value & 0xff;
  AUSB_LOGI(
      "HID GET_REPORT: report_type=%u report_id=%u", report_type_u8, report_id);

  const auto report_type = static_cast<HidReportType>(report_type_u8);
  if (report_type == HidReportType::Input) {
    auto report_queue = get_report_queue(report_id);
    if (!report_queue) {
      AUSB_LOGW("received GET_REPORT request for unknown HID input report %d",
                report_id);
      return pipe->new_in_handler<StallCtrlIn>(pipe);
    }

    // TODO
  } else {
    // TODO: ask a callback to respond to the request
  }

  AUSB_LOGE("TODO: HID GET_REPORT");
  // TODO
  return pipe->new_in_handler<StallCtrlIn>(pipe);
}

CtrlInXfer *HidInterface::get_idle(MessagePipe *pipe,
                                   const SetupPacket &packet) {
  const uint8_t report_id = packet.value & 0xff;
  AUSB_LOGI("HID GET_IDLE: report_id=%u", report_id);

  auto report_queue = get_report_queue(report_id);
  if (!report_queue) {
    AUSB_LOGW("received GET_IDLE request for unknown HID report %d", report_id);
    return pipe->new_in_handler<StallCtrlIn>(pipe);
  }

  auto duration_4ms = report_queue->get_idle_4ms();
  return pipe->new_in_handler<SendU8>(pipe, duration_4ms);
}

CtrlInXfer *HidInterface::get_protocol(MessagePipe *pipe,
                                       const SetupPacket &packet) {
  AUSB_LOGI("HID GET_PROTOCOL");
  return pipe->new_in_handler<SendU8>(pipe, static_cast<uint8_t>(protocol_));
}

} // namespace ausb::hid
