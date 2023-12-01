// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"

namespace ausb {

class UsbDevice;

/**
 * A class for implementing endpoint 0 control OUT transfers in device mode.
 *
 * Thread safety:
 * not thread safe.  ack() and error() should only be called from the main USB
 * task.  Similarly, destructor may only be called from main USB task.
 *
 * If the caller does processing on another task, they should use custom events
 * to trigger the final ack() or error() calls.
 *
 * could implement a thread-safe version:
 * - destructor:
 *   - may need to trigger error() if it was not called.
 *     generally bad if caller does not call ack() or error() first,
 *     since a call to transfer_cancelled() can race with the destructor.
 *
 * - ack() and error():
 *   - generate a
 *
 * - RxDataReference destructor:
 *   - inform the device more buffer space is available from any task
 *
 * - transfer_cancelled() may be called in the USB task while ack() or error()
 *   are running in another task
 *
 * - UsbDevice destructor:
 *   - needs to call transfer_cancelled() on this object
 *     - also needs to wait for RxDataReference objects to be freed :-(
 *
 * -----
 * optimizations:
 * - many CtrlOut transfers complete immediately, with no actual OUT packets
 *   (SET_ADDRESS, SET_CONFIGURATION, SET_FEATURE).
 *   could avoid creating a DevCtrlOutTransfer object in these cases.
 */
class DevCtrlOutTransfer {
public:
  explicit DevCtrlOutTransfer(UsbDevice *device);

  virtual ~DevCtrlOutTransfer();

  /**
   * Send an acknowledgement to the host, indicating that the transfer was
   * successful.
   *
   * This should only be called after all OUT data has been received.
   */
  void ack();

  /**
   * Send an acknowledgement to the host, indicating that the transfer was
   * successful.
   *
   * This should only be called after all OUT data has been received.
   */
  void error();

#if 0
  /**
   * out_data_available will be called when OUT data is received from the host.
   *
   * This method will always be invoked on the main USB task.
   *
   * The RxDataReference will point to packet data that has been received in
   * the endpoint's RX buffer.  is_last will be set to true if this contains
   * the final data for the transfer, or true if we still expect to receive
   * more data from the host.
   *
   * The DevCtrlOutTransfer implementation should generally process this data
   * and destroy the RxDataReference object as soon as possible, to free the RX
   * buffer space.  The RX buffer space may need to be freed before more data
   * can be received from the host, in cases where the entire OUT transfer is
   * too large to fit in the endpoint's RX buffer.
   */
  virtual void out_data_available(RxDataReference data, bool is_last) = 0;
#endif

  /**
   * transfer_cancelled() will be called if the transfer is cancelled.
   *
   * This method will always be invoked on the main USB task.
   *
   * The DevCtrlOutTransfer implementation should cancel any processing
   * they are doing.  It is not necessary to call ack() or error() (doing so
   * will be a no-op).  The DevCtrlOutTransfer destructor will be called
   * immediately after transfer_cancelled() returns.
   */
  virtual void transfer_cancelled(XferCancelReason reason) = 0;

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

  DevCtrlOutTransfer(DevCtrlOutTransfer const &) = delete;
  DevCtrlOutTransfer &operator=(DevCtrlOutTransfer const &) = delete;

  friend class UsbDevice;
  void invoke_transfer_cancelled(XferCancelReason reason) {
      device_ = nullptr;
      transfer_cancelled(reason);
  }

  // Whether all OUT data has been received yet.
  // This flag is set immediately before out_data_available() is called with
  // the final data.  This flag is always set from the main USB task.
  bool all_data_received_ = false;

  // The pointer to the UsbDevice.
  // This will be reset to null when the transfer is complete (by calling ack()
  // or error()), or once it has been cancelled, immediately before a
  // transfer_cancelled() call.
  UsbDevice *device_{nullptr};
};

} // namespace ausb
