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
 * A class for implementing endpoint 0 control IN transfers in device mode.
 *
 * Note that this class (and UsbDevice in general) is not thread safe. 
 * The implementation should only invoke methods on this class from the main
 * USB task.  e.g.  send_full() and error() may only be called from the main
 * USB task.  When the UsbDevice invokes methods on this class, it will always
 * be done from the main USB task.
 */
class CtrlInXfer {
public:
  explicit CtrlInXfer(ControlEndpoint *endpoint) : endpoint_(endpoint) {}

  /**
   * The lifetime of the CtrlInXfer object is controlled by the
   * ControlEndpoint, and the object will be destroyed when it is no longer
   * needed.
   *
   * One of xfer_acked() or xfer_failed() will be called before the object is
   * destroyed.
   */
  virtual ~CtrlInXfer() {}

  ControlEndpoint *usb() const { return endpoint_; }

  /**
   * Begin processing this transfer.
   *
   * This will be called to start processing the transfer shortly after the
   * CtrlInXfer has been created.  The implementation should usually call
   * send_full() to send the desired IN data to the host.
   */
  virtual void start(const SetupPacket& packet) = 0;

  /**
   * Provide the full response to send to the host.
   *
   * The caller must ensure that the data buffer remains valid until
   * the CtrlInXfer object is destroyed.
   *
   * This method may only be invoked from the USB task.
   */
  void send_full(const void* data, size_t size);

  /*
   * TODO: in the future we could implement a send_partial() API to allow
   * providing just some packets to send without having all data ready at once.
   * This API would require that the partial data size be an exact multiple of
   * the endpoint max packet size.
   */
  // void send_partial(const void* data, size_t size);

  /**
   * Send a STALL error to the host, indicating that the transfer failed.
   */
  void error();

  /**
   * xfer_acked() will be called once the host has received and ACKed the
   * transfer.
   */
  virtual void xfer_acked() = 0;

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

private:
  CtrlInXfer(CtrlInXfer const &) = delete;
  CtrlInXfer &operator=(CtrlInXfer const &) = delete;

  friend class ControlEndpoint;
  void invoke_xfer_failed(XferFailReason reason) {
      endpoint_ = nullptr;
      xfer_failed(reason);
  }

  ControlEndpoint *endpoint_{nullptr};
};

} // namespace device
} // namespace ausb
