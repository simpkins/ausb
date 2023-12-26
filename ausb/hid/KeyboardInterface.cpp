// Copyright (c) 2023, Adam Simpkins
#include "ausb/hid/KeyboardInterface.h"

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

namespace ausb::hid {

void KeyboardInterface::send_report(const uint8_t *data) {
  auto *buf = add_report_prepare(kReportId);
  memcpy(buf, data, sizeof(ReportType));
  add_report_complete(kReportId);
}

bool KeyboardInterface::set_output_report(asel::buf_view data) {
  // TODO
  return true;
}

} // namespace ausb::hid
