// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_config.h"
#include "ausb/ausb_types.h"
#include "ausb/dev/DeviceEvent.h"

#include <chrono>
#include <cstdint>

namespace ausb {

class HWDeviceBase {
public:
  constexpr HWDeviceBase() noexcept = default;

  // If multiple hardware types are supported, then the device methods need to
  // be virtual methods so that we can do runtime selection of the correct
  // hardware implementation to use.  If AUSB_CONFIG_HW_MULTI is not defined
  // then only a single hardware configuration is supported, and we know the
  // only possible method to call at compile time.
#if AUSB_CONFIG_HW_MULTI
  [[nodiscard]] virtual std::error_code init() = 0;
  virtual void reset() = 0;

  virtual DeviceEvent wait_for_event(std::chrono::milliseconds timeout) = 0;

  virtual void set_address(uint8_t address) = 0;
  virtual void set_address_early(uint8_t address) = 0;

  virtual bool configure_ep0(uint8_t max_packet_size) = 0;
  [[nodiscard]] virtual XferStartResult
  start_write(uint8_t endpoint, const void *data, uint32_t size) = 0;
  [[nodiscard]] virtual XferStartResult
  start_read(uint8_t endpoint, void *data, uint32_t size) = 0;

  virtual void stall_control_endpoint(uint8_t endpoint_num) = 0;
#endif

private:
  HWDeviceBase(HWDeviceBase const &) = delete;
  HWDeviceBase &operator=(HWDeviceBase const &) = delete;
};

} // namespace ausb
