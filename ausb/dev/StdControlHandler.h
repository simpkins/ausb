// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/desc/DescriptorMap.h"
#include "ausb/dev/ControlEndpoint.h"

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

  std::unique_ptr<CtrlOutXfer>
  process_out_setup(ControlEndpoint *ep, const SetupPacket &packet) override;
  std::unique_ptr<CtrlInXfer>
  process_in_setup(ControlEndpoint *ep, const SetupPacket &packet) override;

#if 0
  void ctrl_out_xfer_done(ControlEndpoint *ep, CtrlOutXfer *xfer) override;
  void ctrl_in_xfer_done(ControlEndpoint *ep, CtrlInXfer *xfer) override;
#endif

private:
  StdControlHandler(StdControlHandler const &) = delete;
  StdControlHandler &operator=(StdControlHandler const &) = delete;

  std::unique_ptr<CtrlOutXfer>
  process_std_device_out(ControlEndpoint *ep, const SetupPacket &packet);
  std::unique_ptr<CtrlInXfer> process_std_device_in(ControlEndpoint *ep,
                                                    const SetupPacket &packet);

  std::unique_ptr<CtrlOutXfer>
  process_set_configuration(ControlEndpoint *ep, const SetupPacket &packet);
  std::unique_ptr<CtrlInXfer> process_get_descriptor(ControlEndpoint *ep,
                                                     const SetupPacket &packet);

  ControlHandlerCallback* const callback_ = nullptr;
  uint8_t ep0_max_packet_size_ = 64;
};

} // namespace ausb::device
