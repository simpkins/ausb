// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/dev/EndpointZero.h"

#include <asel/buf_view.h>
#include <cinttypes>
#include <optional>

namespace ausb::device {

class StdControlHandlerCallback {
public:
  virtual bool set_configuration(uint8_t config_id) = 0;
  virtual std::optional<asel::buf_view> get_descriptor(uint16_t value,
                                                       uint16_t index) = 0;

  virtual void on_reset(XferFailReason reason) {}
  virtual void on_suspend() {}
  virtual void on_resume() {}
};

/**
 * StdControlHandler processes SETUP requests on the device's default control
 * pipe.
 */
class StdControlHandler : public EndpointZeroCallback {
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
  constexpr explicit StdControlHandler(StdControlHandlerCallback *callback)
      : callback_(callback) {}

  void on_reset(XferFailReason reason) override;
  void on_enum_done(uint8_t max_packet_size) override;
  void on_suspend() override;
  void on_resume() override;

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

  StdControlHandlerCallback *const callback_ = nullptr;
  uint8_t ep0_max_packet_size_ = 64;
};

} // namespace ausb::device
