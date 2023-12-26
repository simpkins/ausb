// Copyright (c) 2023, Adam Simpkins
#include "ausb/hid/HidInEndpoint.h"

#include "ausb/dev/EndpointManager.h"
#include "ausb/log.h"
#include <cassert>

namespace ausb::hid {

uint8_t *HidInEndpoint::add_report_prepare(uint8_t report_id,
                                           bool flush_previous_entries) {
  assert(report_id != kUnusedReportID);
  auto buf = reports_->add_report_get_buffer(report_id, flush_previous_entries);
  if (!buf) {
    log_bad_report_id(report_id);
    return nullptr;
  }
  return buf;
}

void HidInEndpoint::add_report_complete(uint8_t report_id) {
  if (is_xfer_in_progress()) {
    return;
  }
  auto queue = reports_->get_report_queue(report_id);
  if (!queue) {
    log_bad_report_id(report_id);
    return;
  }
  start_xfer(queue, reports_->get_longest_report_size());
}

void HidInEndpoint::start_next_transfer() {
  const auto now = asel::chrono::steady_clock::now();
  auto queue = reports_->get_next_pending_xfer(now);
  if (queue) {
    start_xfer(queue, reports_->get_longest_report_size());
  }
}

void HidInEndpoint::start_xfer(const HidReportQueuePtr &queue,
                               uint16_t longest_report_size) {
  // Determine if we need to send a 0-length packet after we finish the initial
  // transfer.  The transfer must end with a short packet, unless it is the
  // longest possible report.
  const auto report_size = queue.report_size();
  if (report_size == longest_report_size) {
    // We never need a terminating packet after the longest report.
    // The host knows no reports can be longer than this.
    send_zero_length_packet_ = false;
  } else if ((report_size % max_packet_size_) == 0) {
    // If the report size is a multiple of the max packet size,
    // we need a 0-length packet to indicate that this is the end of the
    // transfer.
    send_zero_length_packet_ = true;
  } else {
    send_zero_length_packet_ = false;
  }

  const auto now = asel::chrono::steady_clock::now();
  const auto *buf = queue.send_next_report(now);
  current_xmit_report_id_ = queue.report_id();
  manager_->start_in_write(endpoint_num_, buf, report_size);
}

void HidInEndpoint::log_bad_report_id(uint8_t report_id) const {
  AUSB_LOGE("attempted to use unknown report ID %u on HID endpoint %u",
            report_id,
            endpoint_num_);
}

void HidInEndpoint::on_in_xfer_complete() {
  if (send_zero_length_packet_) {
    // We need to send a 0-length packet to finish the current transfer
    send_zero_length_packet_ = false;
    manager_->start_in_write(endpoint_num_, nullptr, 0);
    return;
  }

  clear_in_progress_xfer();
  start_next_transfer();
}

void HidInEndpoint::on_in_xfer_failed(XferFailReason reason) {
  AUSB_LOGW("IN transfer failed on HID endpoint %u", endpoint_num_);
  send_zero_length_packet_ = false;
  clear_in_progress_xfer();
  start_next_transfer();
}

void HidInEndpoint::clear_in_progress_xfer() {
  auto queue = reports_->get_report_queue(current_xmit_report_id_);
  if (queue) {
    queue->xfer_finished();
  }
  current_xmit_report_id_ = kUnusedReportID;
}

} // namespace ausb::hid
