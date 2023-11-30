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
#if 0
  max_packet_size_ = max_ep0_packet_size;
  fail_control_transfer();

  callbacks_->on_enumerated(max_ep0_packet_size);
#endif
}

void UsbDevice::on_setup_received(const SetupPacket &packet) {
  // Ignore any packets until we have seen a reset.
  if ((state_ & StateMask::Mask) == State::Uninit) {
    AUSB_LOGW("ignoring USB setup packet before reset seen");
    return;
  }

  // Process control request
  process_setup_packet(packet);
}

void UsbDevice::process_setup_packet(const SetupPacket &packet) {
  AUSB_LOGI("USB: SETUP received: request_type=0x%02x request=0x%02x "
            "value=0x%04x index=0x%04x length=0x%04x",
            packet.request_type, packet.request, packet.value, packet.index,
            packet.length);

  // TODO
  AUSB_LOGW("TODO: process setup packet");
}

} // namespace ausb
