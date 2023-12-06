// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"

#include <cstddef>
#include <cstdint>

namespace ausb {

class SetupPacket;

namespace device {

class ControlEndpoint;

/**
 * A class for implementing endpoint 0 control OUT transfers in device mode.
 *
 * Note that this class (and UsbDevice in general) is not thread safe.
 * The implementation should only invoke methods on this class from the main
 * USB task.  e.g.  start_read(), ack(), and error() may only be called
 * from the main USB task.  When the UsbDevice invokes methods on this class,
 * it will always be done from the main USB task.
 *
 * If the implementation does any processing on another task, it should use
 * custom events to trigger the final ack() or error() call to occur on the
 * main USB task.
 */
class CtrlOutXfer {
public:
  explicit CtrlOutXfer(ControlEndpoint *endpoint) : endpoint_(endpoint) {}

  /**
   * The lifetime of the CtrlInXfer object is controlled by the
   * ControlEndpoint, and the object will be destroyed when it is no longer
   * needed.
   *
   * Unless the implementation has called ack() or error() to complete the
   * transfer, xfer_failed() will be called before the object is destroyed.
   */
  virtual ~CtrlOutXfer() {}

  ControlEndpoint *endpoint() const { return endpoint_; }

  /**
   * Begin processing this transfer.
   *
   * This will be called to start processing the transfer shortly after the
   * CtrlOutXfer has been created.  The implementation should usually call
   * start_read() to begin reading the OUT data from the host (if the setup
   * length is non-zero).
   */
  virtual void start(const SetupPacket& packet) = 0;

  /**
   * Start reading data from the host.
   *
   * This method simply initiates the read request.  out_data_received() will
   * be called when the read operation is complete.
   *
   * size should normally be the length field from the SetupPacket in order to
   * read the full transfer data.  That said, it is possible to read the data
   * in smaller chunks if desired, but those chunks must be in multiples of the
   * endpoint 0 maximum packet size.  start_read() can be called with less than
   * the full transfer length as long as size is a multiple of the maximum
   * packet size for endpoint 0.  In this case, out_data_received() will be
   * invoked once the specified size has been received, and you can call
   * start_read() again to read the next set of OUT packets from the transfer.
   * Only a single start_read() call can be in progress at a time--a new
   * start_read() attempt cannot be initiated until out_data_received() has
   * been invoked for the previous read.
   */
  void start_read(void *data, uint32_t size);

  /**
   * Send an acknowledgement to the host, indicating that the transfer was
   * successful.
   *
   * This should only be called after all OUT data has been received.
   */
  void ack();

  /**
   * Send a STALL error to the host, indicating that the transfer failed.
   */
  void error();

  /**
   * After start_read() has been called, out_data_received() will be invoked
   * once the data has been received and placed into the buffer that was given
   * to start_read().
   *
   * The bytes_received parameter will indicate the amount of data received
   * from the host.  This may be less than the amount requested in start_read()
   * if the host signaled the end the transfer by sending a short packet.
   */
  virtual void out_data_received(uint32_t bytes_received) = 0;

  /**
   * xfer_failed() will be called if the transfer fails for any reason outside
   * of the transfer itself calling error().
   *
   * This method will always be invoked on the main USB task.
   * Note that xfer_failed() may be invoked even after ack() has been called,
   * if the host does not accept our acknowledgement packet.
   *
   * The CtrlOutXfer implementation should cancel any processing they are
   * doing.  It is not necessary to call ack() or error() (doing so will be a
   * no-op).  The CtrlOutXfer destructor will be called immediately after
   * xfer_failed() returns.
   */
  virtual void xfer_failed(XferFailReason reason) = 0;

private:
  enum class State {
    // We are still waiting on more OUT data from the host
    ReceivingData,
    // We have received all data, and now need to call ack() or error()
    // to return our status to the host.
    StatusPhase,
    // The transfer is complete and nothing more remains to be done.
    Complete,
    // The transfer has been cancelled.  No more data will be received,
    // and any subsequent ack() or error() call will be ignored.
    Cancelled,
  };

  CtrlOutXfer(CtrlOutXfer const &) = delete;
  CtrlOutXfer &operator=(CtrlOutXfer const &) = delete;

  friend class ControlEndpoint;
  void invoke_xfer_failed(XferFailReason reason) {
      endpoint_ = nullptr;
      xfer_failed(reason);
  }

  // The pointer to the ControlEndpoint.
  // This will be reset to null when the transfer is complete (by calling ack()
  // or error()), or once it has been cancelled, immediately before a
  // xfer_failed() call.
  ControlEndpoint *endpoint_{nullptr};
};

} // namespace device
} // namespace ausb
