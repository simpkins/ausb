// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/SetupPacket.h"

#include <type_traits>
#include <variant>

namespace ausb {

enum NoEventReason {
    // No event occurred before the timeout
    Timeout,
    // An event occurred before the timeout, but it only required some
    // low-level logic to continue processing of an existing transfer
    // and did not generate an event that needs to be visible to the higher
    // level application logic.
    HwProcessing,
};

struct NoEvent {
  explicit constexpr NoEvent(NoEventReason r) : reason(r) {}
  NoEventReason reason;
};
struct BusResetEvent {};
struct SuspendEvent {};
struct ResumeEvent {};

enum class UsbSpeed {
  Low,
  Full,
};

struct BusEnumDone {
  explicit constexpr BusEnumDone(UsbSpeed spd)
      : speed{spd} {}

  UsbSpeed speed = UsbSpeed::Low;
};

struct InXferCompleteEvent {
  explicit constexpr InXferCompleteEvent(uint8_t epnum) : endpoint_num(epnum) {}

  uint8_t endpoint_num{0};
};

struct InXferFailedEvent {
  explicit constexpr InXferFailedEvent(uint8_t epnum) : endpoint_num(epnum) {}

  uint8_t endpoint_num{0};
};

using DeviceEvent =
    std::variant<NoEvent, BusResetEvent, SuspendEvent, ResumeEvent, BusEnumDone,
                 SetupPacket, InXferCompleteEvent, InXferFailedEvent>;
static_assert(std::is_trivially_copyable_v<DeviceEvent>,
              "DeviceEvent must be trivially copyable");

} // namespace ausb
