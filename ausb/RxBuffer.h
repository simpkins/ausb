// Copyright (c) 2023, Adam Simpkins
#pragma once

#include <cstdint>
#include <cstdlib>
#include <limits>

// TODO: allocate the data in a single allocation rather than using vector
#include <vector>

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
  constexpr RxBuffer() noexcept = default;
  RxBuffer(RxBuffer &&) = default;
  RxBuffer &operator=(RxBuffer &&) = default;

  static RxBuffer create(uint8_t endpoint, uint16_t max_packet_size,
                         uint8_t num_packets);

  uint8_t endpoint() const { return endpoint_; }
  uint16_t max_packet_size() const { return max_packet_size_; }

  uint8_t num_free_pkts() {
    return capacity_ - num_pkts_available();
  }
  uint8_t num_pkts_available() const {
    // TODO: this might need to use atomic operations, depending on how we
    // implement interrupt processing
    const auto r = read_index_;
    const auto w = write_index_;
    if (w < r) [[unlikely]] {
      // The write counter has wrapped around, but the read counter hasn't yet.
      return (w + 1 + (std::numeric_limits<decltype(r)>::max() - r));
    }
    return w - r;
  }

private:
  RxBuffer(uint8_t endpoint, uint16_t max_packet_size, uint16_t num_packets);

  uint8_t endpoint_ = 0;
  uint8_t capacity_ = 0;  // Number of packets
  uint16_t max_packet_size_ = 0;
  uint8_t write_index_ = 0;
  uint8_t read_index_ = 0;

  // TODO: allocate all of the RxBuffer data in a single allocation
  std::vector<uint8_t> buffer_;
  std::vector<uint16_t> sizes_;
};

} // namespace ausb
