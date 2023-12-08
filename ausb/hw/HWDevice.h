// Copyright (c) 2023, Adam Simpkins
#pragma once

#ifdef ESP_TARGET
#include "ausb/hw/esp/Esp32Device.h"

namespace ausb { using HWDevice = Esp32Device; }
#else
#include "ausb/hw/mock/MockDevice.h"

namespace ausb { using HWDevice = MockDevice; }
#endif
