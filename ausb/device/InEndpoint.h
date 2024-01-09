// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"
#include "ausb/device/ControlMessageHandler.h"

namespace ausb::device {

class InEndpoint : public ControlMessageHandler {
public:
  constexpr InEndpoint() = default;

  /**
   * on_in_ep_unconfigured() will be called when the endpoint is unconfigured.
   *
   * This can happen when the bus is reset, or if SET_CONFIGURE is called to
   * change the device configuration.
   */
  virtual void on_in_ep_unconfigured(XferFailReason reason) {}

  virtual void on_in_xfer_complete() = 0;
  virtual void on_in_xfer_failed(XferFailReason reason) = 0;

private:
  InEndpoint(InEndpoint const &) = delete;
  InEndpoint &operator=(InEndpoint const &) = delete;
};

} // namespace ausb::device
