// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"
#include "ausb/desc/InterfaceDescriptor.h"
#include "ausb/dev/Interface.h"
#include "ausb/hid/HidInEndpoint.h"
#include "ausb/hid/HidReportMap.h"
#include "ausb/hid/types.h"

namespace ausb::device {
class EndpointManager;
}

namespace ausb::hid {

class HidInterfaceCallback {
public:
  [[nodiscard]] virtual bool set_output_report(asel::buf_view data) = 0;
  [[nodiscard]] virtual bool set_protocol(uint8_t protocol) = 0;
};

class HidInterface : public device::Interface {
public:
  constexpr HidInterface(device::EndpointManager *manager,
                         uint8_t in_endpoint_num,
                         uint16_t max_packet_size,
                         const uint8_t *report_descriptor,
                         size_t report_descriptor_size,
                         HidReportMap *report_map)
      : report_descriptor_(report_descriptor),
        report_descriptor_size_(report_descriptor_size),
        report_map_(report_map),
        in_endpoint_(manager, in_endpoint_num, max_packet_size, report_map_) {}

  template <size_t S>
  constexpr HidInterface(device::EndpointManager *manager,
                         uint8_t in_endpoint_num,
                         uint16_t max_packet_size,
                         const std::array<uint8_t, S> &report_descriptor,
                         HidReportMap *report_map)
      : HidInterface(manager,
                     in_endpoint_num,
                     max_packet_size,
                     report_descriptor.data(),
                     report_descriptor.size(),
                     report_map) {}

  HidInEndpoint &in_endpoint() {
    return in_endpoint_;
  }

  // TODO: we could add a templatized version of HidInterface which actually
  // contains the storage space for HidReportMap.  This version could provide a
  // version of add_report_prepare() that is templatized on the report ID and
  // does not have to perform a runtime lookup of the report queue.  It could
  // also return a pointer to the correct underlying report data type, rather
  // than a 'uint8_t*' buffer.

  uint8_t *add_report_prepare(uint8_t report_id,
                              bool flush_previous_entries = false) {
    return in_endpoint_.add_report_prepare(report_id, flush_previous_entries);
  }
  void add_report_complete(uint8_t report_id) {
    in_endpoint_.add_report_complete(report_id);
  }

  device::CtrlOutXfer *process_out_setup(device::MessagePipe *pipe,
                                         const SetupPacket &packet) override;
  device::CtrlInXfer *process_in_setup(device::MessagePipe *pipe,
                                       const SetupPacket &packet) override;

  [[nodiscard]] virtual bool set_output_report(asel::buf_view data) = 0;

  /**
   * Return an interface descriptor for a non-boot interface.
   */
  static constexpr InterfaceDescriptor make_interface_descriptor() {
    return InterfaceDescriptor(UsbClass::Hid, 0, 0);
  }

  /**
   * Return an interface descriptor for mouse or keyboard interface that
   * supports the boot protocol.
   */
  static constexpr InterfaceDescriptor
  make_boot_interface_descriptor(HidProtocol protocol) {
    return InterfaceDescriptor(UsbClass::Hid,
                               static_cast<uint8_t>(HidSubclass::Boot),
                               static_cast<uint8_t>(protocol));
  }

private:
  // Note: the HidInterface does not own the storage for report_descriptor_ or
  // report_map_, and simply points to data owned by the caller.
  const uint8_t *const report_descriptor_ = nullptr;
  size_t const report_descriptor_size_ = 0;
  HidReportMap *const report_map_ = nullptr;
  HidInEndpoint in_endpoint_;
};

} // namespace ausb::hid
