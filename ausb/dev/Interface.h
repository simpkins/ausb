// Copyright (c) 2023, Adam Simpkins
#pragma once

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
 */
class Interface {
public:
  constexpr Interface() noexcept = default;
  virtual ~Interface() = default;

  /**
   * Process an OUT SETUP request on the control endpoint that was sent to this
   * interface.
   */
  virtual std::unique_ptr<CtrlOutXfer>
  process_out_setup(ControlEndpoint* ctrl_ep, const SetupPacket &packet) = 0;

  /**
   * Process an IN SETUP request on the control endpoint that was sent to this
   * interface.
   */
  virtual std::unique_ptr<CtrlInXfer>
  process_in_setup(ControlEndpoint* ctrl_ep, const SetupPacket &packet) = 0;

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
