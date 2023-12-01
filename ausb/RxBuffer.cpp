// Copyright (c) 2023, Adam Simpkins
#include "ausb/RxBuffer.h"

namespace ausb {

RxBuffer RxBuffer::create(uint8_t endpoint, uint16_t max_packet_size,
                          uint8_t num_packets) {
  return RxBuffer(endpoint, max_packet_size, num_packets);
}

RxBuffer::RxBuffer(uint8_t endpoint, uint16_t max_packet_size,
                   uint16_t num_packets)
    : endpoint_(endpoint), capacity_(num_packets),
      max_packet_size_(max_packet_size) {
  buffer_.resize(max_packet_size_ * capacity_, 0);
  sizes_.resize(max_packet_size_ * capacity_, 0);
}

} // namespace ausb
