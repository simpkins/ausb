// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ControlEndpoint.h"

namespace ausb::device {

class ControlHandler : public ControlEndpointCallback {
public:
  ControlHandler() = default;

  void on_enum_done(uint8_t max_packet_size) override;

  std::unique_ptr<CtrlOutXfer>
  process_out_setup(const SetupPacket &packet) override;
  std::unique_ptr<CtrlInXfer>
  process_in_setup(const SetupPacket &packet) override;

private:
  ControlHandler(ControlHandler const &) = delete;
  ControlHandler &operator=(ControlHandler const &) = delete;
};

} // namespace ausb::device
