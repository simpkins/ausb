// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"
#include "ausb/dev/Interface.h"
#include "ausb/hid/HidReportQueue.h"

namespace ausb::hid {

class HidInterfaceCallback {
public:
  [[nodiscard]] virtual bool set_output_report(asel::buf_view data) = 0;
  [[nodiscard]] virtual bool set_protocol(uint8_t protocol) = 0;
};

class HidInterface : public device::Interface {
public:
  constexpr HidInterface(const uint8_t *report_descriptor,
                         size_t report_descriptor_size,
                         HidReportMapIntf *report_map)
      : report_descriptor_(report_descriptor),
        report_descriptor_size_(report_descriptor_size),
        report_map_(report_map) {}

  template <size_t S>
  constexpr HidInterface(const std::array<uint8_t, S> &report_descriptor,
                         HidReportMapIntf *report_map)
      : HidInterface(
            report_descriptor.data(), report_descriptor.size(), report_map) {}

  device::CtrlOutXfer *process_out_setup(device::MessagePipe *pipe,
                                         const SetupPacket &packet) override;
  device::CtrlInXfer *process_in_setup(device::MessagePipe *pipe,
                                       const SetupPacket &packet) override;

  [[nodiscard]] virtual bool set_output_report(asel::buf_view data) = 0;

private:
  // Note: the HidInterface does not own the storage for report_descriptor_ or
  // report_map_, and simply points to data owned by the caller.
  const uint8_t *const report_descriptor_ = nullptr;
  size_t const report_descriptor_size_ = 0;
  HidReportMapIntf *const report_map_ = nullptr;
};

} // namespace ausb::hid
