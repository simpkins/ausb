// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"

#include <cinttypes>
#include <memory>

namespace ausb {

class SetupPacket;

namespace device {

class CtrlInXfer;
class CtrlOutXfer;
class UsbDevice;
class ControlEndpoint;

class ControlEndpointCallback {
public:
  /**
   * set_endpoint() will be called during device initialization, before any
   * other callback methods are invoked.
   */
  void set_endpoint(ControlEndpoint *ep) { endpoint_ = ep; }

  virtual void on_reset(XferFailReason reason) {}
  virtual void on_enum_done(uint8_t max_packet_size) {}

  virtual std::unique_ptr<CtrlOutXfer>
  process_out_setup(const SetupPacket &packet) = 0;
  virtual std::unique_ptr<CtrlInXfer>
  process_in_setup(const SetupPacket &packet) = 0;

protected:
  ControlEndpoint* endpoint_ = nullptr;
};

/**
 * A class to manage transfers on a control endpoint.
 *
 * This class primarily keeps track of the state of the current transfer.
 * Processing of the transfers themselves is done by a separate
 * ControlEndpointCallback object.
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

  constexpr ControlEndpoint(UsbDevice *usb, ControlEndpointCallback *callback,
                            uint8_t endpoint_num = 0)
      : usb_(usb), endpoint_num_(endpoint_num), callback_(callback) {}
  ~ControlEndpoint();

  UsbDevice* usb() const { return usb_; }
  uint8_t number() const { return endpoint_num_; }
  Status status() const { return status_; }

  ////////////////////////////////////////////////////////////////////
  // Methods to be invoked by UsbDevice to inform us of events
  ////////////////////////////////////////////////////////////////////

  void on_init();

  /**
   * on_enum_done() will be called by the UsbDevice after the device is
   * enumerated on the bus.
   *
   * max_packet_size will be the maximum packet size chosen for this endpoint.
   */
  void on_enum_done(uint8_t max_packet_size);

  /**
   * on_reset() should be called by the UsbDevice when the bus is reset.
   *
   * reason should generally be either BusReset or LocalReset.
   */
  void on_reset(XferFailReason reason);

  /**
   * on_setup_received() should be called by the UsbDevice when a SETUP packet is
   * received.
   */
  void on_setup_received(const SetupPacket &packet);

  /**
   * on_in_xfer_complete() should be called by the UsbDevice when a IN transfer
   * started with UsbDevice::start_ctrl_in_write() has finished.
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

  // Return the current CtrlInXfer, and reset the state to Idle
  std::unique_ptr<CtrlInXfer> extract_in_xfer();
  // Return the current CtrlOutXfer, and reset the state to Idle
  std::unique_ptr<CtrlOutXfer> extract_out_xfer();

  UsbDevice* const usb_ = nullptr;

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
    std::unique_ptr<CtrlOutXfer> out;
    // Out is set during IN transfers (status in In*)
    std::unique_ptr<CtrlInXfer> in;
  } xfer_;

  SetupRetransmitDetector setup_rxmit_detector_;
  ControlEndpointCallback* callback_ = nullptr;
};

} // namespace device
} // namespace ausb