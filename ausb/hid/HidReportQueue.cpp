// Copyright (c) 2023, Adam Simpkins
#include "ausb/hid/HidReportQueue.h"

#include "ausb/log.h"
#include <cassert>

namespace ausb::hid {

uint8_t HidReportQueueImpl::add_report_get_index(uint8_t *storage,
                                                 uint16_t report_size,
                                                 uint8_t num_entries,
                                                 bool flush_queue) {
  if (flush_queue) {
    // If we have been asked to drop all pending but unsent entries,
    // just reset next_to_send_ to be unset.
    next_to_send_ = 0xff;
  }

  // Write at 1 past the current state index
  auto write_index = next_queue_index(current_index_, num_entries);
  if (write_index == current_xmit_index_) {
    // We can't overwrite the entry that is currently being transmitted,
    // so in this case we have to advance one more.
    write_index = next_queue_index(current_xmit_index_, num_entries);
  }

  if (write_index == next_to_send_) {
    // next_to_send_ points to the oldest entry in the queue that we have
    // yet to transmit.
    //
    // If we just overwrote this location, our queue is full and we have
    // dropped an entry.  We have to advance next_to_send_.
    AUSB_LOGI("HID report queue is full: dropped oldest untransmitted report");
    next_to_send_ = next_queue_index(next_to_send_, num_entries);
    if (next_to_send_ == current_xmit_index_) {
      // Always have to skip over the current_xmit_index_.
      // Note that this will always happen if num_entries is 2.
      next_to_send_ = next_queue_index(current_xmit_index_, num_entries);
    }
  } else if (next_to_send_ == 0xff) {
    next_to_send_ = write_index;
  }

  current_index_ = write_index;
  return current_index_;
}

const uint8_t *HidReportQueueImpl::send_next_report(
    const uint8_t *storage,
    uint16_t report_size,
    uint8_t num_entries,
    asel::chrono::steady_clock::time_point now) {
  assert(current_xmit_index_ == 0xff);
  if (next_to_send_ != 0xff) {
    current_xmit_index_ = next_to_send_;
    if (next_to_send_ == current_xmit_index_) {
      next_to_send_ = 0xff;
    } else {
      next_to_send_ = next_queue_index(next_to_send_, num_entries);
    }
  } else {
    current_xmit_index_ = current_index_;
  }
  time_last_sent_ = now;
  return storage + (current_xmit_index_ * report_size);
}

} // namespace ausb::hid
