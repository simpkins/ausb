// Copyright (c) 2023, Adam Simpkins
#include "ausb/hid/kbd/BootKeyboard.h"

#include "ausb/SetupPacket.h"
#include "ausb/desc/types.h"
#include "ausb/device/CtrlOutXfer.h"
#include "ausb/device/MessagePipe.h"
#include "ausb/device/ctrl/StallCtrlOut.h"
#include "ausb/hid/types.h"

#include "ausb/log.h"

#include <cstring>

using namespace ausb::device;
using namespace ausb::hid;

namespace ausb::kbd {

class BootKeyboard::SetLedHandler : public device::CtrlOutXfer {
public:
  SetLedHandler(MessagePipe *pipe, BootKeyboard *kbd)
      : CtrlOutXfer(pipe), kbd_(kbd) {}

  void start(const SetupPacket &packet) override {
    start_read(&data_, sizeof(data_));
  }
  void out_data_received(uint32_t bytes_received) override {
    if (bytes_received == 1) {
      AUSB_LOGV("set keyboard LEDs: 0x%02x", data_);
      kbd_->set_leds(data_);
      ack();
    } else {
      // It's possible we could get 0-bytes on a short transfer
      AUSB_LOGW("unexpected length received for keyboard LEDs: %" PRIu32,
                bytes_received);
      error();
    }
  }
  void xfer_failed(XferFailReason reason) override {}

private:
  uint8_t data_ = 0;
  BootKeyboard* kbd_ = nullptr;
};

void BootKeyboard::send_report(const uint8_t *data) {
  auto *buf = add_report_prepare(kReportId);
  memcpy(buf, data, sizeof(ReportType));
  add_report_complete(kReportId);
}

CtrlOutXfer *BootKeyboard::set_report(MessagePipe *pipe,
                                const SetupPacket &packet,
                                HidReportType report_type,
                                uint8_t report_id) {
  if (report_type != HidReportType::Output && report_id == 0) {
    AUSB_LOGE("received SET_REPORT request for unknown report (%u, %u)",
              static_cast<uint8_t>(report_type),
              report_id);
    return pipe->new_out_handler<StallCtrlOut>(pipe);
  }
  if (packet.length != 1) {
    AUSB_LOGE("received SET_REPORT LED request with unexpected length %" PRIu16,
              packet.length);
    return pipe->new_out_handler<StallCtrlOut>(pipe);
  }

  return pipe->new_out_handler<SetLedHandler>(pipe, this);
}

void BootKeyboard::set_leds(uint8_t value) {
  if (callback_) {
    callback_->on_set_leds(value);
  }
}

} // namespace ausb::kbd
