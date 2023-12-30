// Copyright (c) 2023, Adam Simpkins
#include "ausb/hid/kbd/BootKeyboard.h"

#include "ausb/SetupPacket.h"
#include "ausb/desc/types.h"
#include "ausb/dev/CtrlInXfer.h"
#include "ausb/dev/CtrlOutXfer.h"
#include "ausb/dev/MessagePipe.h"
#include "ausb/dev/ctrl/AckEmptyCtrlOut.h"
#include "ausb/dev/ctrl/GetStaticDescriptor.h"
#include "ausb/dev/ctrl/StallCtrlIn.h"
#include "ausb/hid/HidSetReport.h"
#include "ausb/hid/types.h"

#include "ausb/log.h"

#include <cstring>

using namespace ausb::device;
using namespace ausb::hid;

namespace ausb::kbd {

void BootKeyboard::send_report(const uint8_t *data) {
  auto *buf = add_report_prepare(kReportId);
  memcpy(buf, data, sizeof(ReportType));
  add_report_complete(kReportId);
}

bool BootKeyboard::set_output_report(asel::buf_view data) {
  // TODO
  return true;
}

} // namespace ausb::kbd