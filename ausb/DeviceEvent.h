// Copyright (c) 2023, Adam Simpkins
#pragma once

#include <type_traits>
#include <variant>

namespace ausb {

struct UninitializedEvent {};
struct BusResetEvent {};
struct SuspendEvent {};
struct ResumeEvent {};

using DeviceEvent =
    std::variant<UninitializedEvent, BusResetEvent, SuspendEvent, ResumeEvent>;
static_assert(std::is_trivially_copyable_v<DeviceEvent>,
              "DeviceEvent must be trivially copyable");

} // namespace ausb
