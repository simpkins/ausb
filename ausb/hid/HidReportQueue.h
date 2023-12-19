// Copyright (c) 2023, Adam Simpkins
#pragma once

#include <asel/array.h>
#include <asel/chrono.h>
#include <asel/inttypes.h>

#include <chrono>

namespace ausb::hid {

/**
 * HidReportQueue contains a queue of report values to send for a single HID
 * report ID.
 *
 * A report queue must always contain at least 2 entries: one to hold the
 * current report state, and one to hold data currently being transmitted to
 * the host.
 *
 * The ReportSize must include the 1-byte Report ID prefix,
 * if the report will be sent with an ID prefix.
 */
template <uint16_t ReportSize, uint8_t NumEntries>
class HidReportQueue;

class HidReportQueueImpl {
public:
  constexpr explicit HidReportQueueImpl() = default;

  /**
   * Add a new report to the queue, and return index to the queue entry where
   * the new report value should be written.
   *
   * This method may only be called from the main USB task, and the caller
   * should write the report data into the buffer before returning to the USB
   * task loop.
   */
  uint8_t add_report_get_index(uint8_t *storage,
                               uint16_t report_size,
                               uint8_t num_entries,
                               bool flush_queue);

  // start_xmit() returns the index of the entry to send.
  uint8_t start_xmit();
  void xmit_finished();

  bool needs_xfer(asel::chrono::steady_clock::time_point now) const {
    return next_to_send_ != 0xff ||
           (now >= (time_last_sent_ +
                    (idle_rate_in_4ms_ * asel::chrono::milliseconds(4))));
  }

private:
  static uint8_t next_queue_index(uint8_t current_index, uint8_t num_entries) {
    return (current_index + 1) % num_entries;
  }

  // All queue index variables are specified in number of entries,
  // not number of bytes.

  // The index in the queue where the current report state lives.
  uint8_t current_index_ = 0;
  // The index in the queue to the entry we are currently transmitting,
  // or 0xff if we are not currently transmitting an entry.  (This can be 0xff
  // even if we have state to send if the endpoint is currently transmitting a
  // different report.)
  uint8_t current_xmit_index_ = 0xff;
  // The index in the queue to the next entry to transmit to the host,
  // or 0xff if the current report state has already been transmitted.
  uint8_t next_to_send_ = 0xff;

  // The report's idle rate, as configured by a SET_IDLE message.
  // - 0 means indefinite: only send the report when the value changes
  // - Non-zero values are in units of 4ms
  uint8_t idle_rate_in_4ms_ = 10;

  // time_last_sent_ is the time that we most recently sent this report's
  // current state.  If the state has been updated and we have not sent the
  // current value yet, time_last_sent_ will be set to 0.
  //
  // If time_last_sent_ is 0, the report needs to be resent again as soon as
  // possible.  If time_last_sent_ is non-zero and idle_rate_in_4ms_ is
  // non-zero, it needs to be resent in
  // (time_last_sent_ + (idle_rate_in_4ms_ * 4ms))
  //
  // time_last_sent_ should generally be 0 if and only if next_to_send_ and
  // and current_xmit_index_ are both 0xff.
  asel::chrono::steady_clock::time_point time_last_sent_;
};

class HidReportQueuePtr {
public:
  constexpr HidReportQueuePtr() noexcept = default;
  constexpr /* implicit */ HidReportQueuePtr(std::nullptr_t arg) noexcept {}
  HidReportQueuePtr(HidReportQueueImpl *impl,
                    uint16_t report_size,
                    uint8_t num_entries,
                    const uint8_t *storage)
      : impl_(impl),
        report_size_(report_size),
        num_entries_(num_entries),
        storage_(storage) {}

  constexpr explicit operator bool() const {
    return impl_ != nullptr;
  }

private:
  HidReportQueueImpl *impl_ = nullptr;
  uint16_t report_size_ = 0;
  uint8_t num_entries_ = 0;
  const uint8_t *storage_ = nullptr;
};

template <uint16_t ReportSize, uint8_t NumEntries>
class HidReportQueue {
public:
  static constexpr uint16_t report_size = ReportSize;
  static constexpr uint8_t num_entries = NumEntries;
  static_assert(num_entries >= 2,
                "HidReportQueue must contain at least 2 entries");
  static_assert(num_entries < 0xff,
                "HidReportQueue may not contain more than 254 entries");

  explicit constexpr HidReportQueue() = default;

  HidReportQueuePtr get_ptr() {
    return HidReportQueuePtr(&impl_, report_size, num_entries, storage_.data());
  }

  uint8_t *add_report_get_buffer(bool flush_queue) {
    const auto index = impl_.add_report_get_index(
        storage_.data(), report_size, num_entries, flush_queue);
    return &storage_[index * report_size];
  }

  bool needs_xfer(asel::chrono::steady_clock::time_point now) const {
    return impl_.needs_xfer(now);
  }

private:
  asel::array<uint8_t, ReportSize *NumEntries> storage_ = {};
  HidReportQueueImpl impl_;
};

/**
 * A class tracking multiple HID reports, and allowing look up of specific
 * report queues by report ID.
 *
 * The template parameters should generally be ReportInfo objects.
 */
template <typename... Reports>
class HidReportMap {};

template <typename ReportT>
class HidReportMap<ReportT> {
public:
  static constexpr size_t kNumReports = 1;
  static constexpr uint16_t kLongestReportSize = ReportT::report_size;

  constexpr explicit HidReportMap() = default;

  constexpr HidReportQueuePtr get_report_queue(uint8_t report_id) {
    if (report_id == ReportT::report_id) {
      return report_.get_ptr();
    }
    return nullptr;
  }

  /**
   * Return the HidReportQueueImpl to send next, if one needs to be sent
   * immediately, or nullptr if no report needs to be transmitted now.
   */
  HidReportQueuePtr
  get_next_pending_xfer(asel::chrono::steady_clock::time_point now) {
    if (report_.needs_xfer(now)) {
      return report_.get_ptr();
    }
    return nullptr;
  }

private:
  HidReportQueue<ReportT::report_size, ReportT::queue_capacity> report_;
};

template <typename Report1, typename... RemainingReports>
class HidReportMap<Report1, RemainingReports...> {
public:
  static constexpr size_t kNumReports = 1 + sizeof...(RemainingReports);
  static constexpr uint16_t kLongestReportSize =
      std::max(static_cast<uint16_t>(Report1::report_size),
               HidReportMap<RemainingReports...>::kLongestReportSize);

  constexpr explicit HidReportMap() = default;

  constexpr HidReportQueuePtr get_report_queue(uint8_t report_id) {
    if (report_id == Report1::report_id) {
      return report_.get_ptr();
    }
    return others_.get_report_queue(report_id);
  }

  HidReportQueuePtr
  get_next_pending_xfer(asel::chrono::steady_clock::time_point now) {
    // TODO: we walk the list of reports in order, and always return the
    // first one that needs transmitting.  This could result in starvation
    // if earlier report IDs have a lot of events and always need to be sent.
    // It might be nice to have a slightly more fair approach somehow.
    if (report_.needs_xfer(now)) {
      return report_.get_ptr();
    }
    return others_.get_next_pending_xfer(now);
  }

private:
  HidReportQueue<Report1::report_size, Report1::queue_capacity> report_;
  HidReportMap<RemainingReports...> others_;
};

class HidReportMapIntf {
public:
  virtual HidReportQueuePtr get_report_queue(uint8_t report_id) = 0;
};

template <typename... Reports>
class HidReportMapVirtual : public HidReportMapIntf {
public:
  HidReportMapVirtual(HidReportMap<Reports...> *map) : map_(map) {}

  HidReportQueuePtr get_report_queue(uint8_t report_id) override {
    return map_->get_report_queue(report_id);
  }

private:
  HidReportMap<Reports...> *const map_ = nullptr;
};

} // namespace ausb::hid
