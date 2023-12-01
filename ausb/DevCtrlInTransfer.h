// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"

#include <cstddef>
#include <cstdint>

namespace ausb {

class UsbDevice;
class SetupPacket;

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

  virtual void start(const SetupPacket& packet) = 0;
  virtual void xfer_cancelled(XferCancelReason reason) = 0;

  /**
   * Provide the full response to send to the host.
   *
   * The caller must ensure that the data buffer remains valid until
   * send_buffer_released() is called.
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
   * send_buffer_released() will be called once the buffer provided to
   * send_full() is no longer required (once it has been copied into hardware
   * buffers to transmit).
   *
   * Note that send_buffer_released() may be called from inside the invocation to
   * send_full(), before send_full() returns.
   *
   * DevCtrlInTransfer implementations may ignore this call if the buffer they
   * provide will be valid for the lifetime of the DevCtrlInTransfer object.
   */
  virtual void send_buffer_released() {}

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
