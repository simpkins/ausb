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

/*
 * A note about SETUP packet events:
 *
 * The USB host may retransmit SETUP packets if it thinks there was a
 * transmission error.  The SETUP packet does not contain any sort of packet or
 * transaction ID to help detect if a received packet is a retransmission or
 * not.
 *
 * If possible, it is recommended for HWDevice implementations to avoid sending
 * a SetupPacketEvent until they have seen both the SETUP packet and then a
 * subsequent IN or OUT token from the host, to ensure that this SETUP packet
 * is not a retransmission.  If the hardware provides IN NAK and OUT NAK
 * interrupts, these should be used to detect the start of the data phase.
 * SETUP packets received before the first IN or OUT NAK event should be
 * stored, and if multiple SETUP packets are received only the last should be
 * stored and delivered in a SetupPacketEvent.
 *
 * Note that we push the responsibility for this down to the HWDevice layer
 * since some hardware implementations (e.g., Synposys cores) provide this
 * functionality automatically.
 *
 * If the underlying hardware cannot provide this functionality (e.g., if it
 * does not deliver interrupts on IN/OUT NAK), then it is acceptable to deliver
 * SETUP packet events without retransmission detection, but this may result in
 * control transfer processing starting on the first SETUP packet receipt, then
 * being cancelled and restarted when a retransmitted SETUP is received.
 */
struct SetupPacketEvent {
  explicit constexpr SetupPacketEvent(const SetupPacket &p) : pkt(p) {}

  SetupPacket pkt;
};

using DeviceEvent =
    std::variant<NoEvent, BusResetEvent, SuspendEvent, ResumeEvent, BusEnumDone,
                 SetupPacketEvent, InXferCompleteEvent, InXferFailedEvent>;
static_assert(std::is_trivially_copyable_v<DeviceEvent>,
              "DeviceEvent must be trivially copyable");

} // namespace ausb
