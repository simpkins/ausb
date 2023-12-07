// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/dev/CtrlOutXfer.h"

namespace ausb::device {

/**
 * A CtrlOutXfer handler that simply fails the request with a STALL error.
 */
class StallCtrlOut : public CtrlOutXfer {
public:
  using CtrlOutXfer::CtrlOutXfer;

  void start(const SetupPacket &packet) override;
  void out_data_received(uint32_t bytes_received) override;
  void xfer_failed(XferFailReason reason) override;
};

} // namespace ausb::device