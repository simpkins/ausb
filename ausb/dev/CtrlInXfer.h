// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"

#include <cstddef>
#include <cstdint>

namespace ausb {

class SetupPacket;

namespace device {

class MessagePipe;

/**
 * A class for implementing control IN transfers in device mode.
 *
 * Note that this class (and all of the USB device code in general) is not
 * thread safe.  The implementation should only invoke methods on this class
 * from the main USB task.  e.g.  send_full() and error() may only be called
 * from the main USB task.  When methods are invoked on this class due to
 * hardware events, it will always be done from the main USB task.
 */
class CtrlInXfer {
public:
  explicit CtrlInXfer(MessagePipe *pipe) : pipe_(pipe) {}

  /**
   * The lifetime of the CtrlInXfer object is controlled by the
   * MessagePipe, and the object will be destroyed when it is no longer
   * needed.
   *
   * One of xfer_acked() or xfer_failed() will be called before the object is
   * destroyed.
   */
  virtual ~CtrlInXfer() {}

  MessagePipe *pipe() const {
    return pipe_;
  }

  /**
   * Begin processing this transfer.
   *
   * This will be called to start processing the transfer shortly after the
   * CtrlInXfer has been created.  The implementation should usually call
   * send_full() to send the desired IN data to the host.
   */
  virtual void start(const SetupPacket &packet) = 0;

  /**
   * Provide the full response to send to the host.
   *
   * The caller must ensure that the data buffer remains valid until
   * xfer_acked() has been called (or until the CtrlInXfer object is destroyed).
   *
   * This method may only be invoked from the USB task.
   */
  void send_full(const void *data, size_t size) {
    send_final(data, size);
  }

  /**
   * Provide partial response data to send to the host.
   *
   * The size must be an exact multiple of the message pipe's maximum packet
   * size.  After a call to send_partial(), partial_write_complete() will be
   * called by the pipe once the data has been transmitted.  Only a single
   * write attempt may be in progress at a time: no new send_partial() or
   * send_final() call can be made until partial_write_complete() has been
   * invoked.
   *
   * The caller must ensure that the data buffer remains valid until
   * partial_write_complete() has been invoked.
   *
   * send_final() should be used to send the final portion of the data to the
   * host.
   */
  void send_partial(const void *data, size_t size);

  /**
   * Provide the final chunk of the response data to send to the host.
   *
   * After send_partial() has been used to send partial chunks of data to the
   * host, send_final() can be used to send the final portion of data.
   *
   * The caller must ensure that the data buffer remains valid until
   * xfer_acked() has been called (or until the CtrlInXfer object is destroyed).
   */
  void send_final(const void *data, size_t size);

  /**
   * Send a STALL error to the host, indicating that the transfer failed.
   */
  void error();

  /**
   * xfer_acked() will be called once the host has received and ACKed the
   * transfer.
   */
  virtual void xfer_acked() {}

  /**
   * xfer_failed() will be called if the transfer fails for any reason outside
   * of the transfer itself calling error().
   *
   * This method will always be invoked on the main USB task.
   *
   * The CtrlOutXfer implementation should cancel any processing they are
   * doing.  It is not necessary to call ack() or error() (doing so will be a
   * no-op).  The CtrlOutXfer destructor will be called immediately after
   * xfer_failed() returns.
   */
  virtual void xfer_failed(XferFailReason reason) = 0;

  /**
   * After send_partial() has been invoked, partial_write_complete() will be
   * invoked when this partial write has finished.
   */
  virtual void partial_write_complete() {}

private:
  CtrlInXfer(CtrlInXfer const &) = delete;
  CtrlInXfer &operator=(CtrlInXfer const &) = delete;

  friend class MessagePipe;
  void invoke_xfer_failed(XferFailReason reason) {
    pipe_ = nullptr;
    xfer_failed(reason);
  }

  MessagePipe *pipe_{nullptr};
};

} // namespace device
} // namespace ausb
