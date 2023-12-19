// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/dev/InEndpoint.h"
#include "ausb/hid/HidReportQueue.h"

#include <asel/array.h>
#include <asel/chrono.h>
#include <asel/inttypes.h>
#include <chrono>

namespace ausb::device {
class EndpointManager;
}

namespace ausb::hid {

/**
 * A small helper class to provide info about a HID report,
 * for use constructing HidInEndpoint objects.
 */
template <uint8_t ReportID, typename DataType, uint8_t QueueCapacity = 2>
class ReportInfo {
public:
  // Report ID 0 is reserved by the standard.
  // If report_id is 0, then this means that this interface only supports a
  // single report, and so the report ID is not used.
  static constexpr uint8_t report_id = ReportID;

  static constexpr uint8_t queue_capacity = QueueCapacity;

  // The report size, in bytes.
  // HID report contents are defined on a bit-by-bit basis, and the underlying
  // report data may not end on an even byte boundary.  However, reports are
  // always padded to a full byte boundary when sending them to the host.
  static constexpr uint16_t report_size = sizeof(DataType);
};

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

  device::CtrlOutXfer *process_out_setup(device::MessagePipe *pipe,
                                         const SetupPacket &packet,
                                         HidReportMapIntf &report_map);
  device::CtrlInXfer *process_in_setup(device::MessagePipe *pipe,
                                       const SetupPacket &packet,
                                       HidReportMapIntf &report_map);

private:
  HidInEndpointImpl(HidInEndpointImpl const &) = delete;
  HidInEndpointImpl &operator=(HidInEndpointImpl const &) = delete;

  uint8_t endpoint_num_ = 0;
  bool xfer_in_progress_ = false;
  bool send_zero_length_packet_ = false;
  uint16_t max_packet_size_ = 0;
  device::EndpointManager *manager_ = nullptr;
};

template <typename... Reports>
class HidInEndpoint : public device::InEndpoint {
public:
  static constexpr size_t kNumReports = sizeof...(Reports);

  constexpr HidInEndpoint(uint8_t endpoint_num,
                          uint16_t max_packet_size) noexcept
      : impl_(endpoint_num, max_packet_size) {}

  uint8_t *add_report_prepare(uint8_t report_id,
                              bool flush_previous_entries = false) {
    auto *queue = reports_.get_report_queue(report_id);
    if (!queue) {
      impl_.log_bad_report_id(report_id);
      return nullptr;
    }
    return queue->add_report_prepare(report_id, flush_previous_entries);
  }
  void add_report_complete(uint8_t report_id) {
    if (impl_.is_xfer_in_progress()) {
      return;
    }
    auto *queue = reports_.get_report_queue(report_id);
    if (!queue) {
      impl_.log_bad_report_id(report_id);
      return;
    }
    impl_.start_xfer(queue, reports_.kLongestReportSize);
  }

  /////////////////////
  // InEndpoint methods
  /////////////////////
  device::CtrlOutXfer *process_out_setup(device::MessagePipe *pipe,
                                         const SetupPacket &packet) override {
    auto map = HidReportMapVirtual(&reports_);
    return impl_.process_out_setup(pipe, packet, map);
  }
  device::CtrlInXfer *process_in_setup(device::MessagePipe *pipe,
                                       const SetupPacket &packet) override {
    auto map = HidReportMapVirtual(&reports_);
    return impl_.process_in_setup(pipe, packet, map);
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
    auto queue = reports_.get_next_pending_xfer(now);
    if (queue) {
      impl_.start_xfer(queue, reports_.kLongestReportSize);
    }
  }

  HidReportMap<Reports...> reports_;
  HidInEndpointImpl impl_;
};

} // namespace ausb::hid
