// Copyright (c) 2023, Adam Simpkins
#pragma once

#include <cstdint>
#include <memory>

namespace ausb {

/**
 * A receive buffer for a USB endpoint.
 *
 * Note that the RxBuffer class is not thread safe.  Packet data is placed into
 * the buffer from within an interrupt context, but the read and write indices
 * are only updated from the main USB task.
 *
 * - on receive:
 *   - if no more space, enable NAK on the endpoint
 *
 * - on release buffer space
 *   - if needed, increment amount of data allowed to receive, and disable NAK
 *
 * from interrupt ctx, need to be able to atomically add packet and tell if
 *   buffer is full
 * from main task, need to be able to atomically release packet and tell if
 *   buffer was full
 */
class RxBuffer {
public:
  std::unique_ptr<RxBuffer> create(uint8_t endpoint, uint16_t max_packet_size,
                                   uint8_t num_packets);

  uint8_t endpoint() const { return endpoint_; }
  uint16_t max_packet_size() const { return max_packet_size_; }

  uint8_t num_free_pkts() const {
    // FIXME
    return 1;
  }

private:
  RxBuffer(uint8_t endpoint, uint16_t max_packet_size, uint16_t num_packets);
  RxBuffer(RxBuffer const &) = delete;
  RxBuffer &operator=(RxBuffer const &) = delete;

  uint8_t endpoint_ = 0;
  uint8_t capacity_ = 0;  // Number of packets
  uint16_t max_packet_size_ = 0;
  uint8_t write_index_ = 0;
  uint8_t read_index_ = 0;
  uint8_t* buffer_;
  uint16_t* sizes_;
};

} // namespace ausb
