// Copyright (c) 2023, Adam Simpkins
#include "ausb/hid/KeyboardInterface.h"

#include "ausb/SetupPacket.h"
#include "ausb/desc/types.h"
#include "ausb/dev/CtrlInXfer.h"
#include "ausb/dev/CtrlOutXfer.h"
#include "ausb/dev/ctrl/AckEmptyCtrlOut.h"
#include "ausb/dev/ctrl/GetStaticDescriptor.h"
#include "ausb/dev/ctrl/StallCtrlIn.h"
#include "ausb/hid/HidSetReport.h"
#include "ausb/hid/types.h"

#include "ausb/log.h"

using namespace ausb::device;

namespace ausb::hid {

CtrlOutXfer *KeyboardInterface::process_out_setup(ControlEndpoint *ctrl_ep,
                                                  const SetupPacket &packet) {
  const auto req_type = packet.get_request_type();
  if (req_type == SetupReqType::Class) {
      if (packet.request == static_cast<uint8_t>(HidRequest::SetIdle)) {
        const uint8_t duration = (packet.value >> 8) & 0xff;
        const uint8_t report_id = packet.value & 0xff;
        // TODO: actually record and act on the idle setting
        (void)duration;
        (void)report_id;
        return ctrl_ep->new_out_handler<AckEmptyCtrlOut>(ctrl_ep);
      } else if (packet.request ==
                 static_cast<uint8_t>(HidRequest::SetReport)) {
        // TODO
        return ctrl_ep->new_out_handler<HidSetReport>(ctrl_ep, this);
      }

      AUSB_LOGW("unhandled HID request %u to HID keyboard interface",
                packet.request);
      return nullptr;
  }

  AUSB_LOGW(
      "unhandled control request (0x%#02x 0x%#02x) to HID keyboard interface",
      packet.request_type, packet.request);
  return nullptr;
}

CtrlInXfer *KeyboardInterface::process_in_setup(ControlEndpoint *ctrl_ep,
                                                const SetupPacket &packet) {
  const auto req_type = packet.get_request_type();
  if (req_type == SetupReqType::Standard) {
    const auto std_req_type = packet.get_std_request();
    if (std_req_type == StdRequestType::GetDescriptor) {
        if (packet.value == desc_setup_value(DescriptorType::HidReport, 0)) {
          return ctrl_ep->new_in_handler<GetStaticDescriptor>(
              ctrl_ep, report_descriptor_.data());
        } else {
          AUSB_LOGI("GET_DESCRIPTOR request for non-existent HID class "
                    "descriptor 0x%04x",
                    packet.value);
          return ctrl_ep->new_in_handler<StallCtrlIn>(ctrl_ep);
        }
    }
  } else if (req_type == SetupReqType::Class) {
    // TODO
  }

  // TODO:
  AUSB_LOGE("TODO: implement HID Keyboard ctrl IN handling");
  return nullptr;
}

bool KeyboardInterface::set_report(asel::buf_view data) {
  // TODO
  return true;
}

} // namespace ausb::hid
