// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/CtrlOutXfer.h"

namespace ausb::device {

class SetAddress : public CtrlOutXfer {
public:
  using CtrlOutXfer::CtrlOutXfer;

  void start(const SetupPacket &packet) override;
  void out_data_received(uint32_t bytes_received) override;
  void xfer_failed(XferFailReason reason) override;
};

} // namespace ausb::device
