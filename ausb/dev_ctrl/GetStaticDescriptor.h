// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/DevCtrlInTransfer.h"
#include "ausb/log.h"

#include <array>

namespace ausb {

/**
 * Respond to a GET_DESCRIPTOR request with a static buffer whose lifetime we
 * do not need to manage.
 */
class GetStaticDescriptor : public DevCtrlInTransfer {
public:
  GetStaticDescriptor(UsbDevice *dev, const void *data, size_t size)
      : DevCtrlInTransfer(dev), data_(data), size_(size) {}
  template <size_t S>
  GetStaticDescriptor(UsbDevice *dev, const std::array<uint8_t, S> &data)
      : GetStaticDescriptor(dev, data.data(), data.size()) {}

  void start(const SetupPacket &packet) override {
    // It's unexpected if we try to respond with more data than was requested.
    // This could be caused by a badly behaving host, but it might indicate a
    // bug in our device code somewhere.
    if (size_ > packet.length) {
      AUSB_LOGW("data provided for GET_DESCRIPTOR response is longer than host "
                "requested.  descriptor will be truncated.");
      send_full(data_, packet.length);
    } else {
      send_full(data_, size_);
    }
  }

  void xfer_cancelled(XferCancelReason reason) override {
    AUSB_LOGI("GET_DESCRIPTOR xfer cancelled");
  }
  void xfer_acked() override {
    AUSB_LOGI("GET_DESCRIPTOR xfer acked");
  }
  void xfer_failed() override {
    AUSB_LOGI("GET_DESCRIPTOR xfer failed");
  }

private:
  const void *const data_ = nullptr;
  size_t const size_ = 0;
};

} // namespace ausb
