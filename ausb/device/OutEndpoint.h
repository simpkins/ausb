// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"
#include "ausb/device/ControlMessageHandler.h"

namespace ausb::device {

class OutEndpoint : public ControlMessageHandler {
public:
  constexpr OutEndpoint() = default;

  /**
   * on_out_ep_unconfigured() will be called when the endpoint is unconfigured.
   *
   * This can happen when the bus is reset, or if SET_CONFIGURE is called to
   * change the device configuration.
   */
  virtual void on_out_ep_unconfigured(XferFailReason reason) {}

  virtual void on_out_xfer_complete(uint32_t bytes_read) = 0;
  virtual void on_out_xfer_failed(XferFailReason reason) = 0;

private:
  OutEndpoint(OutEndpoint const &) = delete;
  OutEndpoint &operator=(OutEndpoint const &) = delete;
};

} // namespace ausb::device
