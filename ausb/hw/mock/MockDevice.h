// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"
#include "ausb/dev/DeviceEvent.h"
#include "ausb/hw/HWDeviceBase.h"

#include <array>
#include <chrono>

namespace ausb {

/**
 * A mock hardware implementation, for use in unit tests.
 *
 * This allows unit tests to be run on any platform, including on a Linux host.
 */
class MockDevice : public HWDeviceBase {
public:
  // The USB 2.0 spec allows up to 15 in and 15 out endpoints for full speed
  // devices.  For now we don't have a use case for that many.
  static constexpr size_t kMaxOutEndpoints = 6;
  static constexpr size_t kMaxInEndpoints = 6;

  template <typename BufType> struct EndpointState {
    void reset() {
      max_packet_size = 0;
      xfer_in_progress = false;
      stalled = false;
      cur_xfer_data = nullptr;
      cur_xfer_size = 0;
    }
    uint16_t max_packet_size = 0;
    bool xfer_in_progress = false;
    bool stalled = false;
    BufType cur_xfer_data = nullptr;
    size_t cur_xfer_size = 0;
  };
  using OutEndpointState = EndpointState<void*>;
  using InEndpointState = EndpointState<const void*>;

  constexpr MockDevice() noexcept = default;

  ////////////////////////////////////////////////////////////////////
  // HWDeviceBase APIs
  ////////////////////////////////////////////////////////////////////

  [[nodiscard]] std::error_code init();
  void reset();

  DeviceEvent wait_for_event(std::chrono::milliseconds timeout);

  void set_address(uint8_t address);
  void set_address_early(uint8_t address);

  bool configure_ep0(uint8_t max_packet_size);
  [[nodiscard]] XferStartResult start_write(uint8_t endpoint, const void *data,
                                            uint32_t size);
  [[nodiscard]] XferStartResult start_read(uint8_t endpoint, void *data,
                                           uint32_t size);

  void stall_control_endpoint(uint8_t endpoint_num);

  std::array<OutEndpointState, kMaxOutEndpoints> out_eps;
  std::array<InEndpointState, kMaxInEndpoints> in_eps;

  ////////////////////////////////////////////////////////////////////
  // Methods to be invoked by test code
  ////////////////////////////////////////////////////////////////////

  DeviceEvent complete_in_xfer(uint8_t endpoint_num);
  DeviceEvent complete_out_xfer(uint8_t endpoint_num, int32_t bytes_read = -1);
  void reset_in_stall(uint8_t endpoint_num);
  void reset_out_stall(uint8_t endpoint_num);

private:
  MockDevice(MockDevice const &) = delete;
  MockDevice &operator=(MockDevice const &) = delete;
};

} // namespace ausb
