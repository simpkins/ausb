// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"
#include "ausb/dev/DeviceEvent.h"

#include <chrono>

namespace ausb {

/**
 * A mock hardware implementation, for use in unit tests.
 *
 * This allows unit tests to be run on any platform, including on a Linux host.
 */
class MockDevice {
public:
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

private:
  MockDevice(MockDevice const &) = delete;
  MockDevice &operator=(MockDevice const &) = delete;
};

} // namespace ausb
