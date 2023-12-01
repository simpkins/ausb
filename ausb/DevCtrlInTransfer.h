// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"

namespace ausb {

class UsbDevice;

class DevCtrlInTransfer {
public:
  explicit DevCtrlInTransfer(UsbDevice *device) : device_(device) {}
  virtual ~DevCtrlInTransfer() {}

  virtual void transfer_cancelled(XferCancelReason reason) = 0;

private:
  DevCtrlInTransfer(DevCtrlInTransfer const &) = delete;
  DevCtrlInTransfer &operator=(DevCtrlInTransfer const &) = delete;

  friend class UsbDevice;
  void invoke_transfer_cancelled(XferCancelReason reason) {
      device_ = nullptr;
      transfer_cancelled(reason);
  }

  UsbDevice *device_{nullptr};
};

} // namespace ausb
