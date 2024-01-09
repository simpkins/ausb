// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"

#include <cinttypes>

namespace ausb {
class SetupPacket;
}

namespace ausb::device {

class EndpointManager;
class ControlMessageHandler;
class CtrlInXfer;
class CtrlOutXfer;

/**
 * The implementation of a Message Pipe.
 *
 * Message pipes are defined by section 5.3.2.2 of the USB 2.0 spec.
 *
 * This class primarily keeps track of the state of the current transfer, and
 * invokes the message handler on receipt of SETUP, IN, or OUT packets on the
 * pipe.  A ControlMessageHandler will be informed of new SETUP messages, and
 * the CtrlInXfer and CtrlOutXfer classes will be used to complete a single
 * control transfer.
 *
 * Control transfers basically act like RPC calls from the host to the device.
 * A message pipe can only have a single transfer in progress at a time.
 *
 * In practice message pipes are almost always used solely for endpoint 0
 * (the "Default Control Pipe").  In theory the USB spec allows other endpoints
 * to also be configured as message pipes.  However, in practice this is very
 * uncommon.  Not all device hardware supports control transfers on endpoints
 * other than endpoint 0.
 */
class MessagePipe {
public:
  /**
   * The current transfer state of the pipe.
   *
   * Only a single transfer can be in progress at a time.
   */
  enum class Status : uint8_t {
    // No control transfer in progress
    Idle,
    // We have received a SETUP packet for an OUT transfer, and are currently
    // processing that transfer.
    OutXfer,
    // We have completed processing an OUT transfer are sending the final
    // 0-length IN transaction to acknowledge the transfer.
    OutAck,
    // We have received a SETUP packet for an IN transfer,
    // but have not yet prepared IN data to send to the host.
    InSetupReceived,
    // We received a SETUP packet for an IN transfer, and are currently sending
    // data, but more data remains to be sent after the current write.
    InSendPartial,
    // We received a SETUP packet for an IN transfer,
    // and are currently sending the last portion of data.
    InSendFinal,
    // We have sent all data for an IN transfer, and are waiting for the host
    // to acknowledge the transfer.
    InStatus,
  };

  constexpr MessagePipe(EndpointManager *mgr,
                        uint8_t endpoint_num,
                        ControlMessageHandler *handler) noexcept
      : manager_(mgr), handler_(handler), endpoint_num_(endpoint_num) {}
  ~MessagePipe();

  EndpointManager *manager() const {
    return manager_;
  }
  ControlMessageHandler *handler() const {
    return handler_;
  }
  uint8_t endpoint_num() const {
    return endpoint_num_;
  }
  Status status() const {
    return status_;
  }

  ////////////////////////////////////////////////////////////////////
  // Methods to be invoked by EndpointManager to inform us of events
  ////////////////////////////////////////////////////////////////////

  /**
   * on_unconfigured() will be invoked when the endpoint is unconfigured.
   *
   * The most common cause for this is a bus reset, or if the USB device state
   * is reset locally.  For endpoints other than endpoint 0, this can also
   * occur due to a SET_CONFIGURATION call on endpoint 0.
   */
  void on_unconfigured(XferFailReason reason);

  /**
   * on_setup_received() should be called by the EndpointManager when a SETUP
   * packet is received.
   */
  void on_setup_received(const SetupPacket &packet);

  /**
   * on_in_xfer_complete() should be called by the EndpointManager when a IN
   * transfer started with EndpointManager::start_ctrl_in_write() has finished.
   */
  void on_in_xfer_complete();
  void on_in_xfer_failed(XferFailReason reason);

  void on_out_xfer_complete(uint32_t bytes_read);
  void on_out_xfer_failed(XferFailReason reason);

  ////////////////////////////////////////////////////////////////////
  // Methods to be invoked by the current CtrlInXfer or CtrlOutXfer
  ////////////////////////////////////////////////////////////////////

  /**
   * Begin receiving data for the current control OUT transfer.
   *
   * This method should only be invoked by the current CtrlOutXfer object.
   */
  void start_out_read(void *data, size_t size);

  /**
   * Acknowledge an OUT transfer as successful.
   *
   * This method should only be invoked by the current CtrlOutXfer object,
   * after it has read and processed the transfer data.
   */
  void ack_out_xfer();

  /**
   * Fail an OUT transfer by returning a STALL error to the host.
   *
   * This method should only be invoked by the current CtrlOutXfer object.
   * Note that this method will immediately destroy the CtrlOutXfer.
   */
  void fail_out_xfer();

  /**
   * Begin receiving data for the current control IN transfer.
   *
   * This method should only be invoked by the current CtrlInXfer object.
   *
   * is_final should be true if this data completes the data to send as part of
   * this SETUP transfer.  is_final should be false if there is more data to
   * send after this. If is_final is false, size must be an exact multiple of
   * the endpoint maximum packet size.  Once the partial write is complete
   * CtrlInXfer::partial_write_complete() will be invoked.  If is_final is true
   * then xfer_acked() is invoked instead once the host acknowledges the data.
   */
  void start_in_write(const void *data, size_t size, bool is_final = true);

  /**
   * Fail an IN transfer by returning a STALL error to the host.
   *
   * This method should only be invoked by the current CtrlInXfer object.
   * Note that this method will immediately destroy the CtrlInXfer.
   */
  void fail_in_xfer();

  template <typename Handler, typename... Args>
  std::enable_if_t<std::is_base_of_v<CtrlInXfer, Handler>, Handler *>
  new_in_handler(Args &&...args) {
    // TODO: each ControlEndpoint should have a fixed storage space for storing
    // the current request handler, rather than doing a heap allocation here.
    return new Handler(std::forward<Args>(args)...);
  }
  template <typename Handler, typename... Args>
  std::enable_if_t<std::is_base_of_v<CtrlOutXfer, Handler>, Handler *>
  new_out_handler(Args &&...args) {
    // TODO: each ControlEndpoint should have a fixed storage space for storing
    // the current request handler, rather than doing a heap allocation here.
    return new Handler(std::forward<Args>(args)...);
  }

private:
  // This class is just a stub for now.  The purpose is just to provide a hook
  // where we can make a compile-time selection based on the current hardware
  // target for whether or not we want to store the current in-progress SETUP
  // packet to decide if a newly-received one is a retransmit or not.  On
  // hardware that provides its own retransmit avoidance mechanism we don't
  // need to store this state.
  class SetupRetransmitDetector {
  public:
    bool is_retransmit(const SetupPacket &packet) {
      return false;
    }
    void on_setup(const SetupPacket &packet) {}
  };

  MessagePipe(MessagePipe const &) = delete;
  MessagePipe &operator=(MessagePipe const &) = delete;

  // Invoke xfer_failed() on the current transaction, flush any pending data to
  // transfer, and set the endpoint to return a STALL error to the host.
  void fail_current_xfer(XferFailReason reason);
  // Invoke xfer_failed() on the current transaction (but does not STALL or
  // flush buffers)
  void invoke_xfer_failed(XferFailReason reason);
  // Destroy the current CtrlInXfer object, and reset the state to Idle
  void destroy_in_xfer();
  // Destroy the current CtrlOutXfer object, and reset the state to Idle
  void destroy_out_xfer();

  EndpointManager *const manager_ = nullptr;
  ControlMessageHandler *const handler_ = nullptr;

  // A message pipe uses 2 endpoint addresses: both an IN and an OUT address.
  // The USB spec requires that a pipe use the same endpoint number in both
  // directions (only the direction bit can differ in the IN vs OUT addresses).
  uint8_t const endpoint_num_ = 0;

  Status status_ = Status::Idle;

  SetupRetransmitDetector setup_rxmit_detector_;

  union CurrentXfer {
    constexpr CurrentXfer() : idle(nullptr) {}
    ~CurrentXfer() {}

    // Idle is set when status_ is Idle
    void *idle;
    // Out is set during OUT transfers (status is Out*)
    CtrlOutXfer *out;
    // Out is set during IN transfers (status in In*)
    CtrlInXfer *in;
  } xfer_;
};

} // namespace ausb::device
