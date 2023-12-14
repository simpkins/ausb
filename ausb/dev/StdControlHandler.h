// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/dev/ControlEndpoint.h"

#include <asel/buf_view.h>
#include <optional>
#include <cinttypes>

namespace ausb::device {

class ControlHandlerCallback {
public:
  virtual bool set_configuration(uint8_t config_id) = 0;
  virtual std::optional<asel::buf_view> get_descriptor(uint16_t value,
                                                       uint16_t index) = 0;
};

/**
 * StdControlHandler processes most standard SETUP requests on the device's
 * default control endpoint.
 */
class StdControlHandler : public ControlEndpointCallback {
public:
  /**
   * Create a new StdControlHandler.
   *
   * The StdControlHandler stores a reference to the DescriptorMap, but does
   * not own it.  It is the caller's responsibility to ensure that the
   * DescriptorMap is valid for as long as the StdControlHandler object is.
   * (Typically the descriptor map is a global singleton, like the
   * StdControlHandler itself.)
   */
  constexpr explicit StdControlHandler(ControlHandlerCallback *callback)
      : callback_(callback) {}

  void on_enum_done(uint8_t max_packet_size) override;

  CtrlOutXfer *process_out_setup(MessagePipe *pipe,
                                 const SetupPacket &packet) override;
  CtrlInXfer *process_in_setup(MessagePipe *pipe,
                               const SetupPacket &packet) override;

private:
  StdControlHandler(StdControlHandler const &) = delete;
  StdControlHandler &operator=(StdControlHandler const &) = delete;

  CtrlOutXfer *process_std_device_out(MessagePipe *pipe,
                                      const SetupPacket &packet);
  CtrlInXfer *process_std_device_in(MessagePipe *pipe,
                                    const SetupPacket &packet);

  CtrlOutXfer *process_set_configuration(MessagePipe *pipe,
                                         const SetupPacket &packet);
  CtrlInXfer *process_get_descriptor(MessagePipe *pipe,
                                     const SetupPacket &packet);

  ControlHandlerCallback* const callback_ = nullptr;
  uint8_t ep0_max_packet_size_ = 64;
};

} // namespace ausb::device
