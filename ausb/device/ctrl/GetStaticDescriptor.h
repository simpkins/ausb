// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/device/CtrlInXfer.h"

#include <array>

namespace ausb::device {

/**
 * Respond to a GET_DESCRIPTOR request with a static buffer whose lifetime we
 * do not need to manage.
 */
class GetStaticDescriptor : public CtrlInXfer {
public:
  GetStaticDescriptor(MessagePipe *pipe, const void *data, size_t size)
      : CtrlInXfer(pipe), data_(data), size_(size) {}
  template <size_t S>
  GetStaticDescriptor(MessagePipe *pipe, const std::array<uint8_t, S> &data)
      : GetStaticDescriptor(pipe, data.data(), data.size()) {}

  void start(const SetupPacket &packet) override;

  void xfer_acked() override;
  void xfer_failed(XferFailReason reason) override;

private:
  const void *const data_ = nullptr;
  size_t const size_ = 0;
};

} // namespace ausb::device
