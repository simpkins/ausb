// Copyright (c) 2024, Adam Simpkins
#pragma once

#include "ausb/SetupPacket.h"
#include "ausb/device/CtrlInXfer.h"
#include "ausb/log.h"

#include <asel/array.h>
#include <cstring>

namespace ausb::device {

/**
 * Send some arbitrary data in an IN control transfer.
 *
 * This class makes a copy of the data, which is kept until the transfer
 * completes.
 */
template<size_t NumBytes>
class SendData : public CtrlInXfer {
public:
  static constexpr size_t kNumBytes = NumBytes;

  SendData(MessagePipe *pipe, const uint8_t *data) : CtrlInXfer(pipe) {
    memcpy(data_.data(), data, data_.size());
  }

  void start(const SetupPacket &packet) override {
    auto const length_to_send =
        packet.length < data_.size() ? packet.length : data_.size();
    send_full(data_.data(), length_to_send);
  }
  void xfer_acked() override {}
  void xfer_failed(XferFailReason reason) override {
    AUSB_LOGW("SendData xfer failed: reason=%d", static_cast<int>(reason));
  }

private:
  asel::array<uint8_t, NumBytes> data_;
};

template<typename IntType>
class SendInt : public SendData<sizeof(IntType)> {
public:
  using type = IntType;
  using Parent = SendData<sizeof(IntType)>;

  SendInt(MessagePipe *pipe, IntType value) : Parent(pipe, &value) {}
};

using SendU8 = SendInt<uint8_t>;

} // namespace ausb::device
