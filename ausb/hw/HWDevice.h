// Copyright (c) 2023, Adam Simpkins
#pragma once

/*
 * This file defines the HWDevice class.
 *
 * The implementation of this class depends on the build configuration:
 * - If only one hardware type is supported, HWDevice is typedef'ed to that
 *   specific hardware device class.  No dynamic dispatch is needed at runtime
 *   to select the correct hardware implementation functions.
 * - If multiple hardware types are supported, HWDevice is a pure virtual base
 *   class which uses virtual methods to choose the correct method at runtime.
 *   Multiple different USB devices with different hardware types may be used
 *   together in the same program.
 */

#include "ausb/hw/HWDeviceBase.h"

#if AUSB_CONFIG_HW_MULTI

namespace ausb { using HWDevice = HWDeviceBase; }

#else // !AUSB_CONFIG_HW_MULTI

#if AUSB_CONFIG_HW_ESP
#include "ausb/hw/esp/Esp32Device.h"
namespace ausb { using HWDevice = Esp32Device; }
#elif AUSB_CONFIG_HW_MOCK
#include "ausb/hw/mock/MockDevice.h"
namespace ausb { using HWDevice = MockDevice; }
#else
#error "No hardware implementation selected!"
#endif

#endif // !AUSB_CONFIG_HW_MULTI
