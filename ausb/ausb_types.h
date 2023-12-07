// Copyright (c) 2023, Adam Simpkins
#pragma once

#include <cstdint>

namespace ausb {

enum class XferFailReason {
  // The USB bus was reset
  BusReset,
  // The device was reset locally (initiated by a local software call, and not
  // by a reset on the bus).
  LocalReset,
  // Unexpected data was received on the bus, putting us into an error state.
  // e.g., if the host sends a different length of data than it specified in
  // the SETUP packet, or if it sends a new SETUP packet in the middle of an
  // existing incomplete transfer.
  ProtocolError,
  // The local software invoked a method when in the wrong state (e.g., trying
  // to write data during an OUT transfer).  This generally shouldn't happen
  // unless there is a software bug.
  SoftwareError,
  // The host sent more data than we expected to receive in an OUT transfer.
  BufferOverrun,
  // A timeout occurred while waiting for the host to send the desired IN/OUT
  // token.
  Timeout,
};

enum class XferStartResult {
  Ok,
  Busy, // An existing write is already in progress for this endpoint
  EndpointNotConfigured,
};

/**
 * The USB device state.
 *
 * Figure 9-1 in the USB 2.0 spec lists the various device states.
 *
 * We do not distinguish between unattached/attached/powered here.
 * The Uninit state captures all of these.
 */
enum class DeviceState : uint8_t {
  Uninit = 0x00,              // Has not seen a bus reset yet
  Default = 0x01,             // Has been reset, but no address assigned yet
  Address = 0x02,             // Address assigned, but not configured
  Configured = 0x03,          // Configuration selected
  SuspendedUninit = 0x10,     // Suspended while in Uninit state
  SuspendedDefault = 0x11,    // Suspended while in Default state
  SuspendedAddress = 0x12,    // Suspended while in Address state
  SuspendedConfigured = 0x13, // Suspended while in Configured state
};

constexpr bool dev_state_is_suspended(DeviceState state) {
  return (static_cast<uint8_t>(state) & 0x10);
}

/**
 * Returns the unsuspended version of a device state.
 *
 * If called with a state that is not suspended, returns that state as-is.
 */
constexpr DeviceState dev_state_unsuspended(DeviceState state) {
  return static_cast<DeviceState>(static_cast<uint8_t>(state) & ~0x10);
}
/**
 * Returns the suspended version of a device state.
 *
 * If called with a state that is already suspended, returns that state as-is.
 */
constexpr DeviceState dev_state_suspended(DeviceState state) {
  return static_cast<DeviceState>(static_cast<uint8_t>(state) | 0x10);
}

} // namespace ausb
