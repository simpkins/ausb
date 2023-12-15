// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/dev/CtrlOutXfer.h"

#include <vector>

namespace ausb::hid {

class HidInterface;

class HidSetReport : public device::CtrlOutXfer {
public:
  HidSetReport(device::MessagePipe *pipe, HidInterface *intf)
      : CtrlOutXfer(pipe), intf_(intf) {}

  void start(const SetupPacket &packet) override;
  void out_data_received(uint32_t bytes_received) override;
  void xfer_failed(XferFailReason reason) override;

private:
  HidInterface *intf_ = nullptr;

  // TODO: we probably should let the interface supply the buffer.
  // There can only be one control request outstanding at a time, so they can
  // just keep a buffer as a member variable.
  std::vector<uint8_t> buf_;
};

} // namespace ausb::hid
