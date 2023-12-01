// Copyright (c) 2023, Adam Simpkins
#include "ausb/UsbDevice.h"

#include "ausb/log.h"

namespace {
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;
}

namespace ausb {

void UsbDevice::handle_event(const DeviceEvent &event) {
  std::visit(overloaded{
                 [this](const NoEvent &) {
                   // Nothing to do.  NoEvent can be returned if
                   // wait_for_event() was called with a timeout and the timeout
                   // expired before an event occured.
                 },
                 [this](const BusResetEvent &) { on_bus_reset(); },
                 [this](const SuspendEvent &) { on_suspend(); },
                 [this](const ResumeEvent &) { on_resume(); },
                 [this](const BusEnumDone &ev) { on_enum_done(ev.speed); },
                 [this](const SetupPacket &pkt) { on_setup_received(pkt); },
             },
             event);
}

void UsbDevice::on_bus_reset() {
  AUSB_LOGI("on_bus_reset");
#if 0
  callbacks_->on_reset();
#endif
}

void UsbDevice::on_suspend() {
  AUSB_LOGI("on_suspend");
  state_ |= StateFlag::Suspended;

  // Do not invoke the on_suspend() callback for suspend events that occur
  // before the first reset has been seen.  The bus suspend state can be seen
  // when first attached to the bus, but this generally isn't really relevant
  // or worth distinguishing from the normal uninitialized state.
  if ((state_ & StateMask::Mask) != State::Uninit) {
#if 0
    callbacks_->on_suspend();
#endif
  }
}

void UsbDevice::on_resume() {
  if ((state_ & StateFlag::Suspended) != StateFlag::Suspended) {
    return;
  }
  AUSB_LOGI("on_resume");
  state_ &= ~StateFlag::Suspended;
  if ((state_ & StateMask::Mask) != State::Uninit) {
#if 0
    callbacks_->on_wakeup();
#endif
  }
}

void UsbDevice::on_enum_done(UsbSpeed speed) {
  AUSB_LOGI("on_enum_done: speed=%d", static_cast<int>(speed));

  state_ = State::Default;
  config_id_ = 0;
  remote_wakeup_enabled_ = false;

  // EP0 max packet size requirements
  // - Must be 8 when low speed
  // - Must be 64 when full speed
  // - May be 8, 16, 32, or 64 when high speed
  uint16_t max_packet_size = (speed == UsbSpeed::Low) ? 8 : 64;
#if 0
  abort_control_transfer();
#endif

  ep0_rx_buffer_ = hw_->create_rx_buffer(/*endpoint_num=*/0, max_packet_size,
                                         /*num_packets=*/2);
  hw_->configure_ep0(&ep0_rx_buffer_);

#if 0
  callbacks_->on_enumerated(max_ep0_packet_size);
#endif
}

void UsbDevice::on_setup_received(const SetupPacket &packet) {
  AUSB_LOGI("SETUP received: request_type=0x%02x request=0x%02x "
            "value=0x%04x index=0x%04x length=0x%04x",
            packet.request_type, packet.request, packet.value, packet.index,
            packet.length);

  // Ignore any packets until we have seen a reset.
  if ((state_ & StateMask::Mask) == State::Uninit) {
    AUSB_LOGW("ignoring USB setup packet before reset seen");
    return;
  }

  // Handle receipt of a new SETUP packet if we think that we are currently
  // still processing an existing control transfer.
  if (ctrl_status_ != CtrlXferStatus::Idle) [[unlikely]] {
    if (ctrl_status_ == CtrlXferStatus::OutSetupReceived ||
        ctrl_status_ == CtrlXferStatus::InSetupReceived) {
      // This may be a retransmitted SETUP packet.  If it is identical to
      // the current SETUP packet we are processing, ignore it.  Otherwise,
      // fail the current transfer and start a new one.
      if (packet == current_ctrl_transfer_) {
        AUSB_LOGI("ignoring retransmitted SETUP packet");
        return;
      }
    }

    // In any other state it is unexpected for us to get a new SETUP packet
    // while we are still processing an existing transfer.  Fail the
    // outstanding transfer, and then fall through and process the new
    // transfer.
    fail_control_transfer(XferCancelReason::ProtocolError);
  }

  // Process the control request
  current_ctrl_transfer_ = packet;
  if (packet.get_direction() == Direction::Out) {
    ctrl_status_ = CtrlXferStatus::OutSetupReceived;
    process_ctrl_out_setup(packet);
  } else {
    ctrl_status_ = CtrlXferStatus::InSetupReceived;
    new (&ctrl_xfer_.in)
        std::unique_ptr<DevCtrlInTransfer>(process_ctrl_in_setup(packet));
    if (!ctrl_xfer_.in) {
      ctrl_xfer_.in.~unique_ptr();
      ctrl_status_ = CtrlXferStatus::Idle;
      stall_ctrl_transfer();
    }
  }
}

void UsbDevice::process_ctrl_out_setup(const SetupPacket &packet) {
  // TODO
  AUSB_LOGW("TODO: process OUT setup packet");
}

class GetDescriptorXfer : public DevCtrlInTransfer {
public:
  GetDescriptorXfer(UsbDevice* dev) : DevCtrlInTransfer(dev) {}

  void start(const SetupPacket& packet) {
    AUSB_LOGI("GET_DESCRIPTOR: value=0x%x index=%u", packet.value,
              packet.index);
  }

  virtual void transfer_cancelled(XferCancelReason reason) {
    AUSB_LOGI("GET_DESCRIPTOR xfer cancelled");
  }
};

std::unique_ptr<DevCtrlInTransfer>
UsbDevice::process_ctrl_in_setup(const SetupPacket &packet) {
  const auto recipient = packet.get_recipient();
  if (recipient == SetupRecipient::Device) {
    const auto req_type = packet.get_request_type();
    if (req_type == SetupReqType::Standard) {
      const auto std_req_type = packet.get_std_request();
      if (std_req_type == StdRequestType::GetDescriptor) {
        AUSB_LOGW("TODO: process GET_DESCRIPTOR request");
        return std::make_unique<GetDescriptorXfer>(this);
      }
    }
  }

  // TODO
  AUSB_LOGW("TODO: unhandled IN setup packet");
  return nullptr;
}

void UsbDevice::fail_control_transfer(XferCancelReason reason) {
  switch (ctrl_status_) {
  case CtrlXferStatus::Idle:
    return;
  case CtrlXferStatus::OutSetupReceived:
  case CtrlXferStatus::OutRecvData:
  case CtrlXferStatus::OutStatus:
  case CtrlXferStatus::OutAck:
    ctrl_xfer_.out->transfer_cancelled(reason);
    ctrl_xfer_.out.~unique_ptr();
    ctrl_status_ = CtrlXferStatus::Idle;
    return;
  case CtrlXferStatus::InSetupReceived:
  case CtrlXferStatus::InSendData:
  case CtrlXferStatus::InStatus:
    ctrl_xfer_.in->transfer_cancelled(reason);
    ctrl_xfer_.in.~unique_ptr();
    ctrl_status_ = CtrlXferStatus::Idle;
    return;
  }

  AUSB_LOGE("unexpected ctrl xfer state %d in fail_control_transfer()",
            static_cast<int>(ctrl_status_));
}

void UsbDevice::stall_ctrl_transfer() {
  // TODO
#if 0
  hw_.stall_in_endpoint(0);
  hw_.stall_out_endpoint(0);
#endif
}

#if 0
void UsbDevice::on_ep0_out_data(void* arg) {
  // allow notification of multiple packets at once.
  // process each packet in a loop
  //
  // - In OutRecvData state:
  //   - If we receive more data than setup wLength, then error.
  //   - If packet is shorter than max packet size, this is the last potion of
  //     data.
  //   - If packet is max packet size and we have received full amount of data
  //     specified in the SETUP wLength, this is the last portion of data
  //   - otherwise, more data is expected
  // - In any other state:
  //   - error
}
#endif

} // namespace ausb
