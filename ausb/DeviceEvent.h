// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/usb_types.h"

#include <type_traits>
#include <variant>

namespace ausb {

struct NoEvent {};
struct BusResetEvent {};
struct SuspendEvent {};
struct ResumeEvent {};

enum class UsbSpeed {
  Low,
  Full,
};

struct BusEnumDone {
  explicit BusEnumDone(UsbSpeed spd)
      : speed{spd} {}

  UsbSpeed speed{UsbSpeed::Low};
};

using DeviceEvent = std::variant<NoEvent, BusResetEvent, SuspendEvent,
                                 ResumeEvent, BusEnumDone, SetupPacket>;
static_assert(std::is_trivially_copyable_v<DeviceEvent>,
              "DeviceEvent must be trivially copyable");

} // namespace ausb
