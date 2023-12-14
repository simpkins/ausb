// Copyright (c) 2023, Adam Simpkins
#include "ausb/dev/EndpointManager.h"

#include "ausb/log.h"

#include <chrono>

using namespace std::chrono_literals;

namespace {
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;
}

namespace ausb::device {

void EndpointManager::loop() {
  while (true) {
    // The timeout value shouldn't really matter here much, since we want to
    // run forever and just retry immediately on timeout.  Set a relatively
    // large value.
    const auto event = hw_->wait_for_event(3600s);
    handle_event(event);
  }
}

void EndpointManager::handle_event(const DeviceEvent &event) {
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
            on_in_xfer_failed(ev.endpoint_num, ev.reason);
          },
          [this](const OutXferCompleteEvent &ev) {
            on_out_xfer_complete(ev.endpoint_num, ev.bytes_read);
          },
          [this](const OutXferFailedEvent &ev) {
            on_out_xfer_failed(ev.endpoint_num, ev.reason);
          },
      },
      event);
}

void EndpointManager::reset() {
  AUSB_LOGW("EndpointManager::reset() called");
  ep0_.on_reset(XferFailReason::LocalReset);
  hw_->reset();
  state_ = DeviceState::Uninit;
  config_id_ = 0;
  remote_wakeup_enabled_ = false;
}

void EndpointManager::on_bus_reset() {
  AUSB_LOGW("on_bus_reset");
  ep0_.on_reset(XferFailReason::BusReset);
}

void EndpointManager::on_suspend() {
  AUSB_LOGI("on_suspend");
  if (is_suspended()) {
    // Ignore spurious suspend events from the hardware for any reason
    return;
  }

  state_ = dev_state_suspended(state_);

  // Do not invoke the on_suspend() callback for suspend events that occur
  // before the first reset has been seen.  The bus suspend state can be seen
  // when first attached to the bus, but this generally isn't really relevant
  // or worth distinguishing from the normal uninitialized state.
  if (state_ != DeviceState::SuspendedUninit) {
    ep0_.on_suspend();
  }
}

void EndpointManager::on_resume() {
  if (!is_suspended()) {
    // Ignore spurious resume events from the hardware for any reason
    return;
  }
  AUSB_LOGI("on_resume");
  state_ = dev_state_unsuspended(state_);
  if (state_ != DeviceState::Uninit) {
    ep0_.on_resume();
  }
}

void EndpointManager::on_enum_done(UsbSpeed speed) {
  AUSB_LOGI("on_enum_done: speed=%d", static_cast<int>(speed));

  state_ = DeviceState::Default;
  config_id_ = 0;
  remote_wakeup_enabled_ = false;

  // EP0 max packet size requirements
  // - Must be 8 when low speed
  // - Must be 64 when full speed
  // - May be 8, 16, 32, or 64 when high speed
  uint8_t max_packet_size = (speed == UsbSpeed::Low) ? 8 : 64;

  hw_->configure_ep0(max_packet_size);
  ep0_.on_enum_done(max_packet_size);
}

void EndpointManager::on_setup_received(const SetupPacketEvent &event) {
  // Ignore any packets until we have seen a reset.
  if (dev_state_unsuspended(state_) == DeviceState::Uninit) {
    AUSB_LOGW("ignoring USB setup packet before reset seen");
    return;
  }

  if (event.endpoint_num == 0) {
    ep0_.on_setup_received(event.packet);
  } else {
    // We could eventually provide APIs for someone to configure a message
    // pipe on an endpoint other than EP0.  For now we don't provide an API yet
    // to configure any other endpoints as message pipes.
    // Using message pipes on endpoints other than EP0 seems pretty uncommon.
    AUSB_LOGW("SETUP packet received on unexpected endpoint %u",
              event.endpoint_num);
  }
}

void EndpointManager::stall_message_pipe(uint8_t endpoint_num) {
  hw_->stall_control_endpoint(endpoint_num);
}

void EndpointManager::on_in_xfer_complete(uint8_t endpoint_num) {
  if (endpoint_num == 0) {
    ep0_.on_in_xfer_complete();
  } else {
    // TODO
    AUSB_LOGE("TODO: on_in_xfer_complete");
  }
}

void EndpointManager::on_in_xfer_failed(uint8_t endpoint_num,
                                        XferFailReason reason) {
  AUSB_LOGE("TODO: on_in_xfer_failed");
  if (endpoint_num == 0) {
    ep0_.on_in_xfer_failed(reason);
  } else {
    // TODO
    AUSB_LOGE("TODO: on_in_xfer_failed");
  }
}

void EndpointManager::on_out_xfer_complete(uint8_t endpoint_num,
                                           uint32_t bytes_read) {
  if (endpoint_num == 0) {
    ep0_.on_out_xfer_complete(bytes_read);
  } else {
    // TODO
    AUSB_LOGE("TODO: on_out_xfer_complete");
  }
}

void EndpointManager::on_out_xfer_failed(uint8_t endpoint_num,
                                         XferFailReason reason) {
  AUSB_LOGE("TODO: on_out_xfer_failed");
  if (endpoint_num == 0) {
    ep0_.on_out_xfer_failed(reason);
  } else {
    // TODO
    AUSB_LOGE("TODO: on_out_xfer_failed");
  }
}

void EndpointManager::set_address(uint8_t address) {
  state_ = DeviceState::Address;
  hw_->set_address(address);
}

void EndpointManager::set_address_early(uint8_t address) {
  hw_->set_address_early(address);
}

bool EndpointManager::add_interface(uint8_t number, Interface *interface) {
  if (interfaces_.size() <= number) {
    interfaces_.resize(number + 1, nullptr);
  }
  if (interfaces_[number] != nullptr) {
    return false;
  }
  interfaces_[number] = interface;
  return true;
}

Interface *EndpointManager::get_interface(uint8_t number) {
  if (number >= interfaces_.size()) {
    return nullptr;
  }
  return interfaces_[number];
}

void EndpointManager::set_configured() {
  state_ = DeviceState::Configured;
}

void EndpointManager::unconfigure() {
  // TODO: close all open endpoints
  AUSB_LOGE("TODO: EndpointManager::unconfigure()");

  state_ = DeviceState::Address;
}

void EndpointManager::start_ctrl_in_write(MessagePipe *pipe,
                                          const void *data, uint32_t size) {
  auto status = hw_->start_write(pipe->endpoint_num(), data, size);
  if (status != XferStartResult::Ok) {
    AUSB_LOGE("error starting control IN transfer: %d",
              static_cast<int>(status));
    pipe->on_in_xfer_failed(XferFailReason::SoftwareError);
    return;
  }
}

void EndpointManager::start_ctrl_in_ack(MessagePipe* pipe) {
  auto status = hw_->start_read(pipe->endpoint_num(), nullptr, 0);
  if (status != XferStartResult::Ok) {
    AUSB_LOGE("error starting receipt of control IN ACK: %d",
              static_cast<int>(status));
    pipe->on_out_xfer_failed(XferFailReason::SoftwareError);
    return;
  }
}

void EndpointManager::start_ctrl_out_read(MessagePipe *pipe, void *data,
                                          uint32_t size) {
  auto status = hw_->start_read(pipe->endpoint_num(), data, size);
  if (status != XferStartResult::Ok) {
    AUSB_LOGE("error starting control OUT transfer: %d",
              static_cast<int>(status));
    pipe->on_out_xfer_failed(XferFailReason::SoftwareError);
    return;
  }
}

void EndpointManager::start_ctrl_out_ack(MessagePipe *pipe) {
  auto status = hw_->start_write(pipe->endpoint_num(), nullptr, 0);
  if (status != XferStartResult::Ok) {
    AUSB_LOGE("error starting control OUT ACK: %d", static_cast<int>(status));
    pipe->on_in_xfer_failed(XferFailReason::SoftwareError);
    return;
  }
}

} // namespace ausb::device
