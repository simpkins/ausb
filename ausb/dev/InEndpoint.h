// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"
#include "ausb/dev/ControlMessageHandler.h"

namespace ausb::device {

class InEndpoint : public ControlMessageHandler {
public:
  constexpr InEndpoint() = default;

  /**
   * unconfigure() will be called when the endpoint is unconfigured.
   *
   * This can happen when the bus is reset, or if SET_CONFIGURE is called to
   * change the device configuration.
   */
  virtual void unconfigure() {}

  virtual void on_in_xfer_complete() = 0;
  virtual void on_in_xfer_failed(XferFailReason reason) = 0;

private:
  InEndpoint(InEndpoint const &) = delete;
  InEndpoint &operator=(InEndpoint const &) = delete;
};

} // namespace ausb::device
