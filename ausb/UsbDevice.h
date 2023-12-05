// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/DeviceEvent.h"
#include "ausb/ausb_types.h"
#include "ausb/desc/DeviceDescriptor.h"

#include <cstdint>
#include <memory>

#include "ausb/CtrlInXfer.h"
#include "ausb/CtrlOutXfer.h"
// TODO: create a separate header for HWDevice
#include "ausb/esp/Esp32Device.h"
namespace ausb { using HWDevice = Esp32Device; }

namespace ausb::device {

class UsbDevice {
public:
  constexpr explicit UsbDevice(HWDevice *hw,
                               const DeviceDescriptor &desc) noexcept
      : hw_(hw), dev_descriptor_(desc) {}

  void handle_event(const DeviceEvent &event);

#if 0
  void release_rx_buffer(uint8_t endpoint, buf_size_t size);
  void ctrl_out_ack();
  void ctrl_out_error();
#endif

  /**
   * Send data for the current control IN transfer.
   *
   * This method should only be invoked by the current CtrlInXfer object.
   */
  void start_ctrl_in_write(const void *data, size_t size);

  /**
   * Fail the current control IN transfer with a STALL error.
   *
   * This method should only be invoked by the current CtrlInXfer object.
   */
  void stall_ctrl_in_transfer();

  /**
   * Begin receiving data for the current control OUT transfer.
   *
   * This method should only be invoked by the current CtrlOutXfer object.
   */
  void start_ctrl_out_read(void *data, size_t size);

  /**
   * Fail the current control OUT transfer with a STALL error.
   *
   * This method should only be invoked by the current CtrlOutXfer object.
   */
  void stall_ctrl_out_transfer();

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

  enum class CtrlXferStatus {
    // No control transfer in progress
    Idle,
    // We have received a SETUP packet for an OUT transfer,
    // but have not begun accepting OUT data packets.
    OutSetupReceived,

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

    // We have received a SETUP packet for an IN transfer,
    // but have not yet prepared IN data to send to the host.
    InSetupReceived,

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

  // Handlers for some standard control requests
  // These are defined as nested classes mostly since that makes it easy for
  // them to access the private state of UsbDevice.
  class SetAddressXfer;

  UsbDevice(UsbDevice const &) = delete;
  UsbDevice &operator=(UsbDevice const &) = delete;

  void on_bus_reset();
  void on_suspend();
  void on_resume();
  void on_enum_done(UsbSpeed speed);
  void on_setup_received(const SetupPacket &packet);
  void on_in_xfer_complete(uint8_t endpoint_num);
  void on_ep0_in_xfer_complete();
  void on_in_xfer_failed(uint8_t endpoint_num);

  std::unique_ptr<CtrlOutXfer>
  process_ctrl_out_setup(const SetupPacket &packet);
  std::unique_ptr<CtrlInXfer>
  process_ctrl_in_setup(const SetupPacket &packet);
  void fail_control_transfer(XferCancelReason reason);

  void stall_ep0();

  std::unique_ptr<CtrlOutXfer>
  process_std_device_out_ctrl(const SetupPacket &packet);
  std::unique_ptr<CtrlInXfer>
  process_std_device_in_ctrl(const SetupPacket &packet);

  State state_ = State::Uninit;
  uint8_t config_id_ = 0;
  bool remote_wakeup_enabled_ = false;

  CtrlXferStatus ctrl_status_ = CtrlXferStatus::Idle;

  HWDevice* hw_ = nullptr;
  union CtrlXfer {
    constexpr CtrlXfer() : idle(nullptr) {}
    ~CtrlXfer() {}

    // Idle is set when ctrl_status_ is Idle
    void* idle;
    // Out is set during OutRecvData and OutStatus states
    std::unique_ptr<CtrlOutXfer> out;
    // Out is set during InSendData and InStatus states
    std::unique_ptr<CtrlInXfer> in;
  } ctrl_xfer_;

#if 0
  // A copy of the SETUP packet for the control transfer currently being
  // processed.  We keep this mainly so we can detect SETUP packet
  // retransmissions, and avoid unnecessarily aborting an then restarting a
  // transfer on SETUP retransmission.
  SetupPacket current_ctrl_transfer_;
#endif
  DeviceDescriptor dev_descriptor_;
};

} // namespace ausb::device
