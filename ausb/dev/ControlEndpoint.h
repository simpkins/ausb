// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"

#include <cinttypes>
#include <type_traits>

namespace ausb {

class SetupPacket;

namespace device {

class ControlEndpoint;
class CtrlInXfer;
class CtrlOutXfer;
class EndpointManager;

// TODO: move to its own dedicated header file?
class ControlMessageHandler {
public:
  virtual ~ControlMessageHandler() = default;

  /**
   * Return a CtrlOutXfer object for handling an OUT control transfer.
   *
   * This should return a handler object returned by calling
   * ep->new_out_handler(), or nullptr if the request is not supported.
   *
   * If nullptr is returned the transfer will be failed by returning a STALL
   * error to the host.
   */
  virtual CtrlOutXfer *process_out_setup(ControlEndpoint *ep,
                                         const SetupPacket &packet) = 0;

  /**
   * Return a CtrlInXfer object for handling an IN control transfer.
   *
   * This should return a handler object returned by calling
   * ep->new_in_handler(), or nullptr if the request is not supported.
   *
   * If nullptr is returned the transfer will be failed by returning a STALL
   * error to the host.
   */
  virtual CtrlInXfer *process_in_setup(ControlEndpoint *ep,
                                       const SetupPacket &packet) = 0;
};

class ControlEndpointCallback : public ControlMessageHandler {
public:
  virtual ~ControlEndpointCallback() = default;

  /**
   * set_endpoint() will be called during device initialization, before any
   * other callback methods are invoked.
   */
  void set_endpoint(ControlEndpoint *ep) { endpoint_ = ep; }

  virtual void on_reset(XferFailReason reason) {}
  virtual void on_enum_done(uint8_t max_packet_size) {}
  virtual void on_suspend() {}
  virtual void on_resume() {}

protected:
  ControlEndpoint* endpoint_ = nullptr;
};

/**
 * A class to manage transfers on a control pipe.
 *
 * This class primarily keeps track of the state of the current transfer, and
 * invokes the transfer handler on receipt of SETUP, IN, or OUT packets on the
 * control endpoint.  Processing of the transfers themselves is done by a
 * separate ControlEndpointCallback object.
 *
 * Control transfers basically act like RPC calls from the host to the device.
 * A control endpoint can only have a single transfer in progress at a time.
 *
 * Endpoint 0 is always a control pipe.  In theory the USB spec allows
 * other endpoints to also be configured as control pipes.  However, in
 * practice this is very uncommon.  If for some reason you do want to configure
 * another endpoint as a control pipe, beware that most interface and endpoint
 * implementations will not expect to receive control messages from additional
 * control pipes: having multiple control pipes would allow multiple control
 * transfers to be in progress simultaneously, and many control message
 * handlers are likely not prepared to support this.  (Supporting multiple
 * outstanding control transfers would likely require additional storage space
 * to store state for the additional in-progress transfers.)
 */
class ControlEndpoint {
public:
  /**
   * The current transfer state of the endpoint.
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

  constexpr ControlEndpoint(EndpointManager *mgr,
                            ControlEndpointCallback *callback,
                            uint8_t endpoint_num = 0)
      : manager_(mgr), endpoint_num_(endpoint_num), callback_(callback) {}
  ~ControlEndpoint();

  EndpointManager* manager() const { return manager_; }
  uint8_t number() const { return endpoint_num_; }
  Status status() const { return status_; }

  ////////////////////////////////////////////////////////////////////
  // Methods to be invoked by EndpointManager to inform us of events
  ////////////////////////////////////////////////////////////////////

  void on_init();

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
  // Methods to be invoked by transfer objects
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
    bool is_retransmit(const SetupPacket &packet) { return false; }
    void on_setup(const SetupPacket& packet) {}
  };

  ControlEndpoint(ControlEndpoint const &) = delete;
  ControlEndpoint &operator=(ControlEndpoint const &) = delete;

  // Invoke xfer_failed() on the current transaction, flush any pending data to
  // transfer, and set the endpoint to return a STALL error to the host.
  void fail_current_xfer(XferFailReason reason);
  // Invoke xfer_failed() on the current transaction (but does not STALL or
  // flush buffers)
  void invoke_xfer_failed(XferFailReason reason);
  void stall();

  // Destroy the current CtrlInXfer object, and reset the state to Idle
  void destroy_in_xfer();
  // Destroy the current CtrlOutXfer object, and reset the state to Idle
  void destroy_out_xfer();

  EndpointManager* const manager_ = nullptr;

  /**
   * The endpoint number is pretty much always 0.
   *
   * The USB spec documents control endpoints and message pipes generically,
   * but in practice there is pretty much only ever a single control endpoint
   * (and single message pipe) on endpoint 0.
   *
   * While some hardware devices allow configuring other endpoints as control
   * endpoints, not all hardware devices support this.  On the host side, there
   * is usually limited support for sending control transfers to endpoints
   * other than endpoint 0.  (libusb does not appear to support this, newer
   * Windows UWP APIs also only appear to support control transfers to the
   * default control endpoint.  WinUsb_ControlTransfer() does allow specifying
   * an alternate interface.)
   */
  uint8_t const endpoint_num_ = 0;
  Status status_ = Status::Idle;

  union CurrentXfer {
    constexpr CurrentXfer() : idle(nullptr) {}
    ~CurrentXfer() {}

    // Idle is set when status_ is Idle
    void* idle;
    // Out is set during OUT transfers (status is Out*)
    CtrlOutXfer *out;
    // Out is set during IN transfers (status in In*)
    CtrlInXfer *in;
  } xfer_;

  SetupRetransmitDetector setup_rxmit_detector_;
  ControlEndpointCallback* callback_ = nullptr;
};

} // namespace device
} // namespace ausb
