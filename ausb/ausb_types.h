// Copyright (c) 2023, Adam Simpkins
#pragma once

namespace ausb {

enum class XferFailReason {
  // The USB bus was reset
  BusReset,
  // The UsbDevice object was reset locally (initiated by a local software
  // call, and not by a reset on the bus).
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

} // namespace ausb
