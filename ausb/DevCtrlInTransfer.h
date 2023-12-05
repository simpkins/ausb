// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"

#include <cstddef>
#include <cstdint>

namespace ausb {

class SetupPacket;
class UsbDevice;

/**
 * A class for implementing endpoint 0 control IN transfers in device mode.
 *
 * Note that this class (and UsbDevice in general) is not thread safe.  When
 * the UsbDevice class invokes methods on this class, it will always be done
 * from the main USB task.
 *
 * When an implementor of DevCtrlInTransfer invokes any methods, like
 * send_full(), this must also be done from the main USB task.
 */
class DevCtrlInTransfer {
public:
  explicit DevCtrlInTransfer(UsbDevice *device) : device_(device) {}

  /**
   * The lifetime of the DevCtrlInTransfer object is controlled by the
   * UsbDevice, and the object will be destroyed when it is no longer needed.
   *
   * One of xfer_cancelled(), xfer_acked(), or xfer_failed() will be called
   * before the object is destroyed.
   */
  virtual ~DevCtrlInTransfer() {}

  UsbDevice *usb() const { return device_; }

  /**
   * Begin processing this transfer.
   *
   * This will be called by UsbDevice to start processing the transfer.
   * The implementation should usually call send_full() to send the desired IN
   * data to the host.
   */
  virtual void start(const SetupPacket& packet) = 0;
  virtual void xfer_cancelled(XferCancelReason reason) = 0;

  /**
   * Provide the full response to send to the host.
   *
   * The caller must ensure that the data buffer remains valid until
   * the DevCtrlInTransfer object is destroyed.
   *
   * This method may only be invoked from the USB task.
   */
  void send_full(const void* data, size_t size);

  /**
   * Send a STALL error to the host, indicating that the transfer failed.
   */
  void error();

  /*
   * TODO: in the future we could implement a send_partial() API to allow
   * providing just some packets to send without having all data ready at once.
   * This API would require that the partial data size be an exact multiple of
   * the endpoint max packet size.
   */
  // void send_partial(const void* data, size_t size);

  /**
   * xfer_acked() will be called once the host has received and ACKed the
   * transfer.
   */
  virtual void xfer_acked() = 0;

  /**
   * xfer_failed() if the host responds to the transfer with a STALL error
   * condition.
   */
  virtual void xfer_failed() = 0;

private:
  DevCtrlInTransfer(DevCtrlInTransfer const &) = delete;
  DevCtrlInTransfer &operator=(DevCtrlInTransfer const &) = delete;

  friend class UsbDevice;
  void invoke_xfer_cancelled(XferCancelReason reason) {
      device_ = nullptr;
      xfer_cancelled(reason);
  }

  UsbDevice *device_{nullptr};
};

} // namespace ausb
