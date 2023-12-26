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

class HidInEndpoint : public device::InEndpoint {
public:
  constexpr HidInEndpoint(device::EndpointManager *manager,
                          uint8_t endpoint_num,
                          uint16_t max_packet_size,
                          HidReportMap *report_map) noexcept
      : manager_(manager),
        reports_(report_map),
        max_packet_size_(max_packet_size),
        endpoint_num_(endpoint_num) {}

  /**
   * Prepare to add a new INPUT report for the specified report ID.
   *
   * This returns the buffer where the report data should be written.
   * This method may only be called from the main USB task, and the caller must
   * write the report data into the buffer and then call add_report_complete()
   * before returning to the USB task loop.
   */
  uint8_t *add_report_prepare(uint8_t report_id,
                              bool flush_previous_entries = false);
  void add_report_complete(uint8_t report_id);

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

  void on_in_xfer_complete() override;
  void on_in_xfer_failed(XferFailReason reason) override;

private:
  static constexpr uint8_t kUnusedReportID = 0xff;

  HidInEndpoint(HidInEndpoint const &) = delete;
  HidInEndpoint &operator=(HidInEndpoint const &) = delete;

  void start_next_transfer();
  void start_xfer(const HidReportQueuePtr &queue, uint16_t longest_report_size);
  bool is_xfer_in_progress() const {
    return current_xmit_report_id_ != kUnusedReportID;
  }
  void log_bad_report_id(uint8_t report_id) const;
  void clear_in_progress_xfer();

  device::EndpointManager *const manager_ = nullptr;
  HidReportMap *const reports_ = nullptr;
  uint16_t const max_packet_size_ = 0;
  uint8_t const endpoint_num_ = 0;
  uint8_t current_xmit_report_id_ = kUnusedReportID;
  bool send_zero_length_packet_ = false;
};

} // namespace ausb::hid
