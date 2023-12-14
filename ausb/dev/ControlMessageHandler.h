// Copyright (c) 2023, Adam Simpkins
#pragma once

namespace ausb {
class SetupPacket;
}

namespace ausb::device {

class MessagePipe;
class CtrlInXfer;
class CtrlOutXfer;

/**
 * The API for objects that can receive SETUP control messages.
 *
 * SETUP messages are always sent over a control pipe (normally endpoint 0),
 * but they can be targeted at other interfaces or endpoints.  This API allows
 * interfaces and endpoints to handle SETUP messages sent to them on endpoint
 * 0.
 */
class ControlMessageHandler {
public:
  constexpr ControlMessageHandler() noexcept = default;
  virtual ~ControlMessageHandler() noexcept = default;

  /**
   * Return a CtrlOutXfer object for handling an OUT control transfer.
   *
   * This should return a handler object returned by calling
   * pipe->new_out_handler(), or nullptr if the request is not supported.
   *
   * If nullptr is returned the transfer will be failed by returning a STALL
   * error to the host.
   */
  virtual CtrlOutXfer *process_out_setup(MessagePipe *pipe,
                                         const SetupPacket &packet) = 0;

  /**
   * Return a CtrlInXfer object for handling an IN control transfer.
   *
   * This should return a handler object returned by calling
   * pipe->new_in_handler(), or nullptr if the request is not supported.
   *
   * If nullptr is returned the transfer will be failed by returning a STALL
   * error to the host.
   */
  virtual CtrlInXfer *process_in_setup(MessagePipe *pipe,
                                       const SetupPacket &packet) = 0;

private:
  ControlMessageHandler(ControlMessageHandler const &) = delete;
  ControlMessageHandler &operator=(ControlMessageHandler const &) = delete;
};

} // namespace ausb::device
