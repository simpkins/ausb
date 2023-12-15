// Copyright (c) 2023, Adam Simpkins
#pragma once

#include <cstdint>
#include <type_traits>

namespace ausb::hid {

/*
 * Usage Page definitions from the HID Usage Tables v1.3 spec
 * https://usb.org/sites/default/files/hut1_3_0.pdf
 *
 * Note that the HUT specification defines one usage page code that does not
 * fit into a uint8_t, but we restrict ourselves to defining only the uint8_t
 * ones here to simplify the HidReportDescriptor::usage_page() API, so that it
 * knows it can encode the value in 1 byte.
 */
enum class UsagePage : uint8_t {
  Undefined = 0x00,
  GenericDesktop = 0x01,
  SimulationControls = 0x02,
  VRControls = 0x03,
  SportControls = 0x04,
  GameControls = 0x05,
  GenericDeviceControls = 0x06,
  KeyCodes = 0x07,
  LEDs = 0x08,
  Button = 0x09,
  Ordinal = 0x0a,
  TelephonyDevice = 0x0b,
  Consumer = 0x0c,
  Digitizers = 0x0d,
  Haptics = 0x0e,
  PhysicalInputDevice = 0x0f,
  Unicode = 0x10,
  EyeAndHeadTrackers = 0x12,
  AuxialiaryDisplay = 0x014,
  Sensors = 0x020,
  MedicalInstrument = 0x40,
  BrailleDisplay = 0x41,
  LightingAndIllumination = 0x59,
  Monitor = 0x80,
  MonitorEnumerated = 0x81,
  VESAVirtualControls = 0x82,
  Power = 0x84,
  BatterySystem = 0x85,
  BarcodeScanner = 0x8c,
  Scales = 0x8d,
  MagneticStripeReader = 0x8e,
  CameraControl = 0x90,
  Arcade = 0x91,
  GamingDevice = 0x92,
  // FIDO Alliance page is 0xF1D0
  // Vendor-defined range is 0xFF00 through 0xFFFF
};

/**
 * A traits class that can be used to indicate that an enum type defines HID
 * usage values.  Each usage page has its own set of usage values, so each page
 * will need its own enum for usage values.
 *
 * This traits type allows ReportDescriptor::usage() to accept only valid usage
 * enum types.
 */
template <typename T>
class is_usage_type : public std::false_type {};

} // namespace ausb::hid
