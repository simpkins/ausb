// Copyright (c) 2023, Adam Simpkins
#include "ausb/UsbDevice.h"

#include "ausb/log.h"
#include "ausb/dev_ctrl/GetStaticDescriptor.h"

namespace {
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;
}

namespace ausb::device {

#if 0
class UsbDevice::SetAddressXfer : public CtrlOutXfer {
public:
  using CtrlOutXfer::CtrlOutXfer;

  void start(const SetupPacket &packet) override {
    if (packet.length > 0) {
      error();
    }
    const uint8_t address = packet.value;
    AUSB_LOGI("SET_ADDRESS: %u", packet.value);
    UsbDevice* usb = endpoint()->usb();
    usb->state_ = UsbDevice::State::Address;
    usb->hw_->set_address(address);
    ack();
  }

  virtual void out_data_received(uint32_t bytes_received) {
    // We never call start_read(), so we don't expect to ever receive data
  }

  void xfer_failed(XferFailReason reason) override {
  }
};
#endif

void UsbDevice::handle_event(const DeviceEvent &event) {
  std::visit(
      overloaded{
          [this](const NoEvent &) {
            // Nothing to do.  NoEvent can be returned if
            // wait_for_event() was called with a timeout and the timeout
            // expired before an event occured.
          },
          [this](const BusResetEvent &) { on_bus_reset(); },
          [this](const SuspendEvent &) { on_suspend(); },
          [this](const ResumeEvent &) { on_resume(); },
          [this](const BusEnumDone &ev) { on_enum_done(ev.speed); },
          [this](const SetupPacketEvent &ev) { on_setup_received(ev); },
          [this](const InXferCompleteEvent &ev) {
            on_in_xfer_complete(ev.endpoint_num);
          },
          [this](const InXferFailedEvent &ev) {
            on_in_xfer_failed(ev.endpoint_num);
          },
      },
      event);
}

void UsbDevice::reset() {
  AUSB_LOGW("UsbDevice::reset() called");
  ep0_.on_reset(XferFailReason::LocalReset);
  hw_->reset();
  state_ = State::Uninit;
  config_id_ = 0;
  remote_wakeup_enabled_ = false;
}

void UsbDevice::on_bus_reset() {
  AUSB_LOGW("on_bus_reset");
  ep0_.on_reset(XferFailReason::BusReset);

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
  uint8_t max_packet_size = (speed == UsbSpeed::Low) ? 8 : 64;

  hw_->configure_ep0(max_packet_size);
  ep0_.on_enum_done(max_packet_size);

#if 0
  callbacks_->on_enumerated(max_ep0_packet_size);
#endif
}

void UsbDevice::on_setup_received(const SetupPacketEvent &event) {
  // Ignore any packets until we have seen a reset.
  if ((state_ & StateMask::Mask) == State::Uninit) {
    AUSB_LOGW("ignoring USB setup packet before reset seen");
    return;
  }

  if (event.endpoint_num == 0) {
    ep0_.on_setup_received(event.packet);
  } else {
    // I can't really think of many practical use cases for someone to
    // configure any endpoint other than EP0 as a control endpoint, but in
    // theory we could potentially support this in the future.
    AUSB_LOGW("SETUP packet received on unexpected endpoint %u",
              event.endpoint_num);
  }
}

void UsbDevice::stall_control_endpoint(uint8_t endpoint) {
  hw_->stall_control_endpoint(endpoint);
}

void UsbDevice::on_in_xfer_complete(uint8_t endpoint_num) {
  if (endpoint_num == 0) {
    ep0_.on_in_xfer_complete();
  } else {
    AUSB_LOGE("TODO: on_in_xfer_complete");
  }
}

void UsbDevice::on_in_xfer_failed(uint8_t endpoint_num) {
  // TODO
  AUSB_LOGE("TODO: on_in_xfer_failed");
}

void UsbDevice::start_ctrl_in_write(ControlEndpoint *endpoint, const void *data,
                                    uint32_t size) {
  auto status = hw_->start_write(endpoint->number(), data, size);
  if (status != XferStartResult::Ok) {
    AUSB_LOGE("error starting control IN transfer: %d",
              static_cast<int>(status));
    endpoint->on_in_xfer_failed(XferFailReason::SoftwareError);
    return;
  }
}

void UsbDevice::start_ctrl_in_ack(ControlEndpoint* endpoint) {
  auto status = hw_->start_read(endpoint->number(), nullptr, 0);
  if (status != XferStartResult::Ok) {
    AUSB_LOGE("error starting receipt of control IN ACK: %d",
              static_cast<int>(status));
    endpoint->on_out_xfer_failed(XferFailReason::SoftwareError);
    return;
  }
}

void UsbDevice::start_ctrl_out_read(ControlEndpoint *endpoint, void *data,
                                    uint32_t size) {
  auto status = hw_->start_read(endpoint->number(), data, size);
  if (status != XferStartResult::Ok) {
    AUSB_LOGE("error starting control OUT transfer: %d",
              static_cast<int>(status));
    endpoint->on_out_xfer_failed(XferFailReason::SoftwareError);
    return;
  }
}

void UsbDevice::start_ctrl_out_ack(ControlEndpoint *endpoint) {
  auto status = hw_->start_write(endpoint->number(), nullptr, 0);
  if (status != XferStartResult::Ok) {
    AUSB_LOGE("error starting control OUT ACK: %d", static_cast<int>(status));
    endpoint->on_in_xfer_failed(XferFailReason::SoftwareError);
    return;
  }
}

#if 0
std::unique_ptr<CtrlOutXfer>
UsbDevice::process_ctrl_out_setup(const SetupPacket &packet) {
  if (packet.request_type ==
      SetupPacket::make_request_type(Direction::Out, SetupRecipient::Device,
                                     SetupReqType::Standard)) {
    return process_std_device_out_ctrl(packet);
    AUSB_LOGE("TODO: process standard device OUT setup packet");
    return nullptr;
  }

  AUSB_LOGE("TODO: process OUT setup packet");
  return nullptr;
}

std::unique_ptr<CtrlInXfer>
UsbDevice::process_ctrl_in_setup(const SetupPacket &packet) {
  if (packet.request_type ==
      SetupPacket::make_request_type(Direction::In, SetupRecipient::Device,
                                     SetupReqType::Standard)) {
    return process_std_device_in_ctrl(packet);
  }

  AUSB_LOGW("TODO: unhandled IN setup packet");
  return nullptr;
}

std::unique_ptr<CtrlOutXfer>
UsbDevice::process_std_device_out_ctrl(const SetupPacket &packet) {
  const auto std_req_type = packet.get_std_request();
  if (std_req_type == StdRequestType::SetAddress) {
    return std::make_unique<SetAddressXfer>(this);
  }

  AUSB_LOGE("TODO: unhandled OUT std device setup packet");
  return nullptr;
}

std::unique_ptr<CtrlInXfer>
UsbDevice::process_std_device_in_ctrl(const SetupPacket &packet) {
  const auto std_req_type = packet.get_std_request();
  if (std_req_type == StdRequestType::GetDescriptor) {
    if (packet.value == 0x0100 && packet.index == 0) {
      AUSB_LOGI("process GET_DESCRIPTOR request for device descriptor");
      return std::make_unique<GetStaticDescriptor>(this,
                                                   dev_descriptor_.data());
    } else {
      AUSB_LOGW("TODO: process GET_DESCRIPTOR request");
    }
  }

  // TODO
  AUSB_LOGW("TODO: unhandled IN std device setup packet");
  return nullptr;
}

void UsbDevice::start_ctrl_in_write(const void *data, size_t size) {
  if (ctrl_status_ == CtrlXferStatus::InSetupReceived) {
    ctrl_status_ = CtrlXferStatus::InSendData;
  } else if (ctrl_status_ == CtrlXferStatus::InSendData) {
    // no state change required
  } else if (ctrl_status_ == CtrlXferStatus::OutSetupReceived ||
             ctrl_status_ == CtrlXferStatus::OutRecvData) {
    // This should be the IN packet signaling success during the status
    // phase of a control OUT transfer
    assert(size == 0);
    ctrl_status_ = CtrlXferStatus::OutStatus;
  } else {
    AUSB_LOGE("start_ctrl_in_write invoked in bad ctrl state %d",
              static_cast<int>(ctrl_status_));
    fail_control_transfer(XferCancelReason::SoftwareError);
    return;
  }

  auto status = hw_->start_write(/*endpoint_num=*/0, data, size);
  if (status != XferStartResult::Ok) {
    AUSB_LOGE("error starting control IN transfer: %d",
              static_cast<int>(status));
    fail_control_transfer(XferCancelReason::SoftwareError);
    return;
  }
}

void UsbDevice::start_ctrl_out_read(void *data, size_t size) {
  if (ctrl_status_ != CtrlXferStatus::OutSetupReceived &&
      ctrl_status_ != CtrlXferStatus::OutRecvData) [[unlikely]] {
    AUSB_LOGE("start_ctrl_in_write invoked in bad ctrl state %d",
              static_cast<int>(ctrl_status_));
    fail_control_transfer(XferCancelReason::SoftwareError);
    return;
  }

  auto status = hw_->start_read(/*endpoint_num=*/0, data, size);
  if (status != XferStartResult::Ok) {
    AUSB_LOGE("error starting control OUT transfer: %d",
              static_cast<int>(status));
    fail_control_transfer(XferCancelReason::SoftwareError);
    return;
  }
}

void UsbDevice::stall_ctrl_in_transfer() {
  if (ctrl_status_ == CtrlXferStatus::InSetupReceived ||
      ctrl_status_ == CtrlXferStatus::InSendData ||
      ctrl_status_ == CtrlXferStatus::InStatus) {
    ctrl_xfer_.in.reset();
    ctrl_xfer_.in.~unique_ptr();
    ctrl_status_ = CtrlXferStatus::Idle;
  } else {
    AUSB_LOGE("stall_ctrl_in_transfer invoked in bad ctrl state %d",
              static_cast<int>(ctrl_status_));
    fail_control_transfer(XferCancelReason::SoftwareError);
  }

  hw_->stall_ep0();
}

void UsbDevice::stall_ctrl_out_transfer() {
  if (ctrl_status_ == CtrlXferStatus::OutSetupReceived ||
      ctrl_status_ == CtrlXferStatus::OutRecvData ||
      ctrl_status_ == CtrlXferStatus::OutStatus ||
      ctrl_status_ == CtrlXferStatus::OutAck) {
    ctrl_xfer_.out.reset();
    ctrl_xfer_.out.~unique_ptr();
    ctrl_status_ = CtrlXferStatus::Idle;
  } else {
    AUSB_LOGE("stall_ctrl_out_transfer invoked in bad ctrl state %d",
              static_cast<int>(ctrl_status_));
    fail_control_transfer(XferCancelReason::SoftwareError);
  }

  hw_->stall_ep0();
}

void UsbDevice::fail_control_transfer(XferCancelReason reason) {
  // TODO: fail_control_transfer() can often be invoked from inside a control
  // transfer method call.  Rather than deleting the control transfer object
  // immediately, it might be better to wait to delete the object until the
  // next iteration around the task loop, to avoid destroying the object while
  // it is still running on the stack.

  switch (ctrl_status_) {
  case CtrlXferStatus::Idle:
    return;
  case CtrlXferStatus::OutSetupReceived:
  case CtrlXferStatus::OutRecvData:
  case CtrlXferStatus::OutStatus:
  case CtrlXferStatus::OutAck:
    ctrl_xfer_.out->invoke_xfer_cancelled(reason);
    ctrl_xfer_.out.reset();
    ctrl_xfer_.out.~unique_ptr();
    ctrl_status_ = CtrlXferStatus::Idle;
    return;
  case CtrlXferStatus::InSetupReceived:
  case CtrlXferStatus::InSendData:
  case CtrlXferStatus::InStatus:
    ctrl_xfer_.in->invoke_xfer_cancelled(reason);
    ctrl_xfer_.in.reset();
    ctrl_xfer_.in.~unique_ptr();
    hw_->flush_tx_fifo(0);
    ctrl_status_ = CtrlXferStatus::Idle;
    return;
  }

  AUSB_LOGE("unexpected ctrl xfer state %d in fail_control_transfer()",
            static_cast<int>(ctrl_status_));
}
#endif

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

} // namespace ausb::device
