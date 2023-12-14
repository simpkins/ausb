// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"
#include "ausb/dev/ControlMessageHandler.h"
#include "ausb/dev/MessagePipe.h"

#include <cinttypes>

namespace ausb::device {

class EndpointManager;

class EndpointZeroCallback : public ControlMessageHandler {
public:
  virtual ~EndpointZeroCallback() = default;

  virtual void on_reset(XferFailReason reason) {}
  virtual void on_enum_done(uint8_t max_packet_size) {}
  virtual void on_suspend() {}
  virtual void on_resume() {}
};

/**
 * This class manages state for the Default Control Pipe (endpoint 0).
 *
 * This class largely consists of the MessagePipe for endpoint 0, plus a few
 * additional device state callbacks that are specific to endpoint 0.
 */
class EndpointZero {
public:
  constexpr EndpointZero(EndpointManager *mgr, EndpointZeroCallback *callback)
      : pipe_(mgr, /*endpoint_num*/ 0, callback) {}
  ~EndpointZero();

  EndpointManager* manager() const { return pipe_.manager(); }
  uint8_t endpoint_num() const { return 0; }
  MessagePipe::Status status() const { return pipe_.status(); }

  ////////////////////////////////////////////////////////////////////
  // Methods to be invoked by EndpointManager to inform us of events
  ////////////////////////////////////////////////////////////////////

  /**
   * on_enum_done() will be called by the EndpointManager after the device is
   * enumerated on the bus.
   *
   * max_packet_size will be the maximum packet size chosen for this endpoint.
   */
  void on_enum_done(uint8_t max_packet_size);

  /**
   * on_reset() should be called by the EndpointManager when the bus is reset.
   *
   * reason should generally be either BusReset or LocalReset.
   */
  void on_reset(XferFailReason reason);

  /**
   * on_suspend() will be invoked when a suspend state is seen on the bus
   * (the bus has been idle for more than 3ms).
   */
  void on_suspend();

  /**
   * on_resume() will be invoked when resumed bus activity is seen after a
   * suspend state.
   */
  void on_resume();

  /**
   * on_setup_received() should be called by the EndpointManager when a SETUP
   * packet is received.
   */
  void on_setup_received(const SetupPacket &packet) {
    pipe_.on_setup_received(packet);
  }

  /**
   * on_in_xfer_complete() should be called by the EndpointManager when a IN
   * transfer started with EndpointManager::start_ctrl_in_write() has finished.
   */
  void on_in_xfer_complete() { pipe_.on_in_xfer_complete(); }
  void on_in_xfer_failed(XferFailReason reason) {
    pipe_.on_in_xfer_failed(reason);
  }

  void on_out_xfer_complete(uint32_t bytes_read) {
    pipe_.on_out_xfer_complete(bytes_read);
  }
  void on_out_xfer_failed(XferFailReason reason) {
    pipe_.on_out_xfer_failed(reason);
  }

private:
  EndpointZero(EndpointZero const &) = delete;
  EndpointZero &operator=(EndpointZero const &) = delete;

  EndpointZeroCallback* get_callback() {
    // We passed our EndpointZeroCallback to the MessagePipe.
    // Rather than storing a second copy of this pointer, just downcast it's
    // handler back to our EndpointZeroCallback type.
    return static_cast<EndpointZeroCallback *>(pipe_.handler());
  }

  MessagePipe pipe_;
};

} // namespace ausb::device
