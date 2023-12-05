// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"

#include <cstddef>
#include <cstdint>

namespace ausb {

class SetupPacket;

namespace device {

class UsbDevice;

/**
 * A class for implementing endpoint 0 control OUT transfers in device mode.
 *
 * Note that this class (and UsbDevice in general) is not thread safe.
 *
 * not thread safe.  ack() and error() should only be called from the main USB
 * task.  Similarly, destructor may only be called from main USB task.
 *
 * If the caller does processing on another task, they should use custom events
 * to trigger the final ack() or error() calls.
 */
class CtrlOutXfer {
public:
  explicit CtrlOutXfer(UsbDevice *device) : device_(device) {}

  virtual ~CtrlOutXfer() {}

  UsbDevice *usb() const { return device_; }

  /**
   * Begin processing this transfer.
   *
   * This will be called by UsbDevice to start processing the transfer.
   * The implementation should usually call start_read() to begin reading the
   * OUT data from the host (if the setup length is non-zero).
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
   * xfer_cancelled() will be called if the transfer is cancelled.
   *
   * This method will always be invoked on the main USB task.
   *
   * The CtrlOutXfer implementation should cancel any processing they are
   * doing.  It is not necessary to call ack() or error() (doing so will be a
   * no-op).  The CtrlOutXfer destructor will be called immediately after
   * xfer_cancelled() returns.
   */
  virtual void xfer_cancelled(XferCancelReason reason) = 0;

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

  friend class UsbDevice;
  void invoke_xfer_cancelled(XferCancelReason reason) {
      device_ = nullptr;
      xfer_cancelled(reason);
  }

  // Whether all OUT data has been received yet.
  // This flag is set immediately before out_data_available() is called with
  // the final data.  This flag is always set from the main USB task.
  bool all_data_received_ = false;

  // The pointer to the UsbDevice.
  // This will be reset to null when the transfer is complete (by calling ack()
  // or error()), or once it has been cancelled, immediately before a
  // xfer_cancelled() call.
  UsbDevice *device_{nullptr};
};

} // namespace device
} // namespace ausb
