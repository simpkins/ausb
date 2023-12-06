// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ControlEndpoint.h"
#include "ausb/desc/DescriptorMap.h"

namespace ausb::device {

class ControlHandler : public ControlEndpointCallback {
public:
  /**
   * Create a new ControlHandler.
   *
   * The ControlHandler stores a reference to the DescriptorMap, but does not
   * own it.  It is the caller's responsibility to ensure that the
   * DescriptorMap is valid for as long as the ControlHandler object is.
   * (Typically the descriptor map is a global singleton, like the
   * ControlHandler itself.)
   */
  constexpr ControlHandler(const DescriptorMap* descriptors)
      : descriptors_(descriptors) {}

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
  std::unique_ptr<CtrlInXfer> process_get_descriptor(const SetupPacket &packet);

  const DescriptorMap* descriptors_;
  uint8_t ep0_max_packet_size_ = 64;
};

} // namespace ausb::device
