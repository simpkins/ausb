// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/dev/CtrlInXfer.h"

namespace ausb::device {

class GetConfiguration : public CtrlInXfer {
public:
  GetConfiguration(MessagePipe *pipe, uint8_t config_id)
      : CtrlInXfer(pipe), config_id_(config_id) {}

  void start(const SetupPacket &packet) override;
  void xfer_acked() override;
  void xfer_failed(XferFailReason reason) override;

private:
  uint8_t config_id_;
};

} // namespace ausb::device
