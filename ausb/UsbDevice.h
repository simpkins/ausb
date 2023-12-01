// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/DeviceEvent.h"

#include <cstdint>

namespace ausb {

class UsbDevice {
public:
  constexpr UsbDevice() noexcept = default;

  void handle_event(const DeviceEvent &event);

#if 0
  void release_rx_buffer(uint8_t endpoint, buf_size_t size);
  void ctrl_out_ack();
  void ctrl_out_error();
#endif

private:
  // Figure 9-1 in the USB 2.0 spec lists the various device states.
  //
  // We do not distinguish between unattached/attached/powered here.
  // The Uninit state captures all of these.
  enum class State : uint8_t {
    Uninit = 0, // Has not seen a bus reset yet
    Default = 1, // has been reset, but no address assigned yet
    Address = 2, // address assigned, but not configured
    Configured = 3,
  };
  // Suspended is a bit flag that can be ANDed with any of the
  // other states.
  enum class StateFlag : uint8_t {
    Suspended = 0x10,
  };
  enum class StateMask : uint8_t {
    Mask = 0x0f,
  };

  enum class CtrlXferState {
      // No control transfer in progress
      Idle,
      // We received a SETUP packet for an OUT transfer,
      // and we are waiting on more OUT data
      OutRecvData,
      // A OUT transfer has been started and we have received all data,
      // and we are not processing the transfer before sending an
      // acknowledgement
      OutStatus,
      // We are acknowledging an OUT transfer.
      // (Unclear if we need this state; in general the HW can receive our
      // 0-length IN packet without waiting for the host to ACK it.)
      OutAck,
      // We received a SETUP packet for an IN transfer,
      // and are currently sending data.
      InSendData,
      // We have sent all data for an IN transfer, and are waiting for the host
      // to acknowledge the transfer.
      InStatus,
  };

  friend State &operator|=(State &s, StateFlag flag) {
    s = static_cast<State>(static_cast<uint8_t>(s) |
                           static_cast<uint8_t>(flag));
    return s;
  }
  friend State &operator&=(State &s, StateFlag flag) {
    s = static_cast<State>(static_cast<uint8_t>(s) &
                           static_cast<uint8_t>(flag));
    return s;
  }
  friend StateFlag operator&(State s, StateFlag flag) {
    return static_cast<StateFlag>(static_cast<uint8_t>(s) &
                                  static_cast<uint8_t>(flag));
  }
  friend StateFlag operator~(StateFlag flag) {
    return static_cast<StateFlag>(~static_cast<uint8_t>(flag));
  }
  friend State operator&(State s, StateMask mask) {
    return static_cast<State>(static_cast<uint8_t>(s) &
                              static_cast<uint8_t>(mask));
  }

  UsbDevice(UsbDevice const &) = delete;
  UsbDevice &operator=(UsbDevice const &) = delete;

  void on_bus_reset();
  void on_suspend();
  void on_resume();
  void on_enum_done(UsbSpeed speed);
  void on_setup_received(const SetupPacket &packet);
  void process_setup_packet(const SetupPacket &packet);

  State state_ = State::Uninit;
  uint8_t config_id_ = 0;
  bool remote_wakeup_enabled_ = false;

  CtrlXferState ctrl_state_ = CtrlXferState::Idle;
#if 0
  union {
    // Out is set during OutRecvData and OutStatus states
    DevCtrlOutTransfer *out;
    // Out is set during InSendData and InStatus states
    DevCtrlInTransfer *in;
  } ctrl_xfer_;
#endif
};

} // namespace ausb
