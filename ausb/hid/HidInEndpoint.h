// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/desc/EndpointDescriptor.h"
#include "ausb/dev/InEndpoint.h"
#include "ausb/hid/HidReportMap.h"

#include <asel/array.h>
#include <asel/chrono.h>
#include <asel/inttypes.h>
#include <chrono>

namespace ausb::device {
class EndpointManager;
}

namespace ausb::hid {

/*
 * This class contains code for the implementation of HidInEndpoint.
 * This is all of the code that does not need to be templatized.  We keep it in
 * a separate class to avoid it being instantiated multiple times, one for each
 * template instantiation.
 */
class HidInEndpointImpl {
public:
  constexpr HidInEndpointImpl(uint8_t endpoint_num,
                              uint16_t max_packet_size) noexcept
      : endpoint_num_(endpoint_num), max_packet_size_(max_packet_size) {}

  void start_xfer(const HidReportQueuePtr &queue, uint16_t longest_report_size);
  bool is_xfer_in_progress() const;
  void log_bad_report_id(uint8_t report_id) const;

  /*
   * Returns true if we should attempt to start a new transfer
   */
  bool on_in_xfer_complete();
  bool on_in_xfer_failed(XferFailReason reason);

private:
  HidInEndpointImpl(HidInEndpointImpl const &) = delete;
  HidInEndpointImpl &operator=(HidInEndpointImpl const &) = delete;

  uint8_t endpoint_num_ = 0;
  bool xfer_in_progress_ = false;
  bool send_zero_length_packet_ = false;
  uint16_t max_packet_size_ = 0;
  device::EndpointManager *manager_ = nullptr;
};

class HidInEndpoint : public device::InEndpoint {
public:
  constexpr HidInEndpoint(uint8_t endpoint_num,
                          uint16_t max_packet_size) noexcept
      : impl_(endpoint_num, max_packet_size) {}

  /**
   * Prepare to add a new INPUT report for the specified report ID.
   *
   * This returns the buffer where the report data should be written.
   * This method may only be called from the main USB task, and the caller must
   * write the report data into the buffer and then call add_report_complete()
   * before returning to the USB task loop.
   */
  uint8_t *add_report_prepare(uint8_t report_id,
                              bool flush_previous_entries = false) {
    auto buf =
        reports_->add_report_get_buffer(report_id, flush_previous_entries);
    if (!buf) {
      impl_.log_bad_report_id(report_id);
      return nullptr;
    }
    return buf;
  }
  void add_report_complete(uint8_t report_id) {
    if (impl_.is_xfer_in_progress()) {
      return;
    }
    auto queue = reports_->get_report_queue(report_id);
    if (!queue) {
      impl_.log_bad_report_id(report_id);
      return;
    }
    impl_.start_xfer(queue, reports_->get_longest_report_size());
  }

  static constexpr EndpointDescriptor make_descriptor(uint8_t endpoint_num,
                                                      uint16_t max_packet_size,
                                                      uint8_t interval = 10) {
    EndpointDescriptor desc;
    desc.set_address(Direction::In, endpoint_num);
    desc.set_type(EndpointType::Interrupt);
    desc.set_interval(interval);
    desc.set_max_packet_size(max_packet_size);
    return desc;
  }

  /////////////////////
  // InEndpoint methods
  /////////////////////
  device::CtrlOutXfer *process_out_setup(device::MessagePipe *pipe,
                                         const SetupPacket &packet) override {
    // HID-class requests are sent to the interface.  We don't expect any SETUP
    // requests to be sent to the endpoint.
    return nullptr;
  }
  device::CtrlInXfer *process_in_setup(device::MessagePipe *pipe,
                                       const SetupPacket &packet) override {
    return nullptr;
  }

  void on_in_xfer_complete() override {
    if (impl_.on_in_xfer_complete()) {
      start_next_transfer();
    }
  }
  void on_in_xfer_failed(XferFailReason reason) override {
    if (impl_.on_in_xfer_failed(reason)) {
      start_next_transfer();
    }
  }

private:
  void start_next_transfer() {
    const auto now = asel::chrono::steady_clock::now();
    auto queue = reports_->get_next_pending_xfer(now);
    if (queue) {
      impl_.start_xfer(queue, reports_->get_longest_report_size());
    }
  }

  HidReportMap *reports_ = nullptr;
  HidInEndpointImpl impl_;
};

} // namespace ausb::hid
