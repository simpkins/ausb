// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/hid/HidReportQueue.h"

#include <asel/chrono.h>

namespace ausb::hid {

/**
 * HidReportMap is a map of Report IDs to HidReportQueue objects.
 *
 * This is used solely for INPUT reports, to implement a HID IN endpoint.
 */
class HidReportMap {
public:
  constexpr HidReportMap() noexcept = default;

  virtual HidReportQueuePtr get_report_queue(uint8_t report_id) = 0;
  virtual uint16_t get_longest_report_size() const = 0;

  /**
   * Return the HidReportQueueImpl to send next, if one needs to be sent
   * immediately, or nullptr if no report needs to be transmitted now.
   */
  virtual HidReportQueuePtr
  get_next_pending_xfer(asel::chrono::steady_clock::time_point now) = 0;

  /**
   * Add a new report to the queue for the specified report ID, and return
   * index to the queue entry where the new report value should be written.
   *
   * This method may only be called from the main USB task, and the caller
   * should write the report data into the buffer before returning to the USB
   * task loop.
   */
  virtual uint8_t *add_report_get_buffer(uint8_t report_id,
                                         bool flush_queue = false) = 0;

private:
  HidReportMap(HidReportMap const &) = delete;
  HidReportMap &operator=(HidReportMap const &) = delete;
};

/**
 * HidReportMapStorage is the actual concrete implementation of HidReportMap.
 *
 * This implements the HidReportMap API, and provides storage for all of the
 * HidReportQueues.
 */
template <typename... Reports>
class HidReportMapStorage {};

template <typename ReportT>
class HidReportMapStorage<ReportT> : public HidReportMap {
public:
  static constexpr size_t kNumReports = 1;
  static constexpr uint16_t kLongestReportSize = ReportT::report_size;

  uint16_t get_longest_report_size() const override final {
    return kLongestReportSize;
  }
  HidReportQueuePtr get_report_queue(uint8_t report_id) override final {
    if (report_id == ReportT::report_id) {
      return report_.get_ptr();
    }
    return nullptr;
  }

  HidReportQueuePtr get_next_pending_xfer(
      asel::chrono::steady_clock::time_point now) override final {
    if (report_.needs_xfer(now)) {
      return report_.get_ptr();
    }
    return nullptr;
  }

  uint8_t *add_report_get_buffer(uint8_t report_id,
                                 bool flush_queue = false) override final {
    if (report_id == ReportT::report_id) {
      return report_.add_report_get_buffer(flush_queue);
    }
    return nullptr;
  }

private:
  HidReportQueue<ReportT::report_size, ReportT::queue_capacity> report_;
};

template <typename Report1, typename... RemainingReports>
class HidReportMapStorage<Report1, RemainingReports...> : public HidReportMap {
public:
  static constexpr size_t kNumReports = 1 + sizeof...(RemainingReports);
  static constexpr uint16_t kLongestReportSize =
      std::max(static_cast<uint16_t>(Report1::report_size),
               HidReportMapStorage<RemainingReports...>::kLongestReportSize);

  uint16_t get_longest_report_size() const override final {
    return kLongestReportSize;
  }
  HidReportQueuePtr get_report_queue(uint8_t report_id) override final {
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

  uint8_t *add_report_get_buffer(uint8_t report_id,
                                 bool flush_queue = false) override final {
    if (report_id == Report1::report_id) {
      return report_.add_report_get_buffer(flush_queue);
    }
    return others_.add_report_get_buffer(report_id, flush_queue);
  }

private:
  HidReportQueue<Report1::report_size, Report1::queue_capacity> report_;
  HidReportMapStorage<RemainingReports...> others_;
};

} // namespace ausb::hid
