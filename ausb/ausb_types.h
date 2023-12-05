// Copyright (c) 2023, Adam Simpkins
#pragma once

namespace ausb {

enum class XferCancelReason {
  // The USB bus was reset
  BusReset,
  // Unexpected data was received on the bus, putting us into an error state.
  // e.g., if the host sends a different SETUP packet while this transfer is in
  // progress, or if it sends an IN packet before sending the full OUT data for
  // this transfer.
  ProtocolError,
  // The UsbDevice object was destroyed.
  UsbDeviceDestroyed,
  // The local software invoked a method when in the wrong state (e.g., trying
  // to write data during an OUT transfer).  This generally shouldn't happen
  // unless there is a software bug.
  SoftwareError,
};

enum class XferStartResult {
  Ok,
  Busy, // An existing write is already in progress for this endpoint
  EndpointNotConfigured,
};

} // namespace ausb
