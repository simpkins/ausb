// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ControlEndpoint.h"
#include "ausb/desc/DeviceDescriptor.h"

namespace ausb::device {

class ControlHandler : public ControlEndpointCallback {
public:
  constexpr ControlHandler(const DeviceDescriptor& dev_desc)
      : dev_descriptor_(dev_desc) {}

  void on_enum_done(uint8_t max_packet_size) override;

  std::unique_ptr<CtrlOutXfer>
  process_out_setup(const SetupPacket &packet) override;
  std::unique_ptr<CtrlInXfer>
  process_in_setup(const SetupPacket &packet) override;

private:
  ControlHandler(ControlHandler const &) = delete;
  ControlHandler &operator=(ControlHandler const &) = delete;

  std::unique_ptr<CtrlOutXfer>
  process_std_device_out(const SetupPacket &packet);
  std::unique_ptr<CtrlInXfer> process_std_device_in(const SetupPacket &packet);

  DeviceDescriptor dev_descriptor_;
};

} // namespace ausb::device
