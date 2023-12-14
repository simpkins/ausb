// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/dev/ControlEndpoint.h"

#include <memory>

namespace ausb {
class SetupPacket;
}

namespace ausb::device {

class ControlEndpoint;
class CtrlInXfer;
class CtrlOutXfer;

/**
 * A pure virtual class defining the API that any device interface must
 * implement.
 *
 * This derives from ControlMessageHandler, since control messages on
 * Endpoint 0 may indicate that they are for a specific interface.  This
 * Interface object will be asked to process any SETUP control messages sent to
 * it on endpoint 0.
 */
class Interface : public ControlMessageHandler {
public:
  constexpr Interface() noexcept = default;
  virtual ~Interface() = default;

  /**
   * unconfigure() will be called when the interface is unconfigured.
   *
   * This can happen when the bus is reset, or if SET_CONFIGURE is called to
   * change the device configuration.
   */
  virtual void unconfigure() {}

private:
  Interface(Interface const &) = delete;
  Interface &operator=(Interface const &) = delete;
};

} // namespace ausb::device
