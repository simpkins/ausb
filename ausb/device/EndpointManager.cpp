// Copyright (c) 2023, Adam Simpkins
#include "ausb/device/EndpointManager.h"

#include "ausb/device/InEndpoint.h"
#include "ausb/device/Interface.h"
#include "ausb/device/OutEndpoint.h"
#include "ausb/log.h"

#include <cassert>
#include <chrono>

using namespace std::chrono_literals;

namespace {
template <class... Ts>
struct overloaded : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;
} // namespace

namespace ausb::device {

void EndpointManager::reset() {
  AUSB_LOGW("EndpointManager::reset() called");
  unconfigure_endpoints_and_interfaces(XferFailReason::LocalReset,
                                       /*include_ep0=*/true);
  ep0_.on_reset(XferFailReason::LocalReset);
  hw_->reset();
  state_ = DeviceState::Uninit;
  remote_wakeup_enabled_ = false;
}

void EndpointManager::on_bus_reset() {
  AUSB_LOGW("on_bus_reset");
  unconfigure_endpoints_and_interfaces(XferFailReason::BusReset,
                                       /*include_ep0=*/true);
  ep0_.on_reset(XferFailReason::BusReset);
  state_ = DeviceState::Uninit;
  remote_wakeup_enabled_ = false;
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
  ep0_.on_enum_done(speed, max_packet_size);
}

void EndpointManager::on_setup_received(uint8_t endpoint_num,
                                        const SetupPacket &packet) {
  // Ignore any packets until we have seen a reset.
  if (dev_state_unsuspended(state_) == DeviceState::Uninit) {
    AUSB_LOGW("ignoring USB setup packet before reset seen");
    return;
  }

  if (endpoint_num == 0) {
    ep0_.on_setup_received(packet);
  } else {
    // We potentially could add an on_setup_received() method to the
    // OutEndpoint API, to support endpoints other than endpoint 0 to be
    // configured as message pipes.  However, there doesn't really seem to be
    // much practical use for message pipes on other endpoints.  For now we
    // don't bother supporting this.
    AUSB_LOGW("SETUP packet received on unexpected endpoint %u", endpoint_num);
  }
}

void EndpointManager::stall_message_pipe(uint8_t endpoint_num) {
  hw_->stall_control_endpoint(endpoint_num);
}

void EndpointManager::on_in_xfer_complete(uint8_t endpoint_num) {
  if (endpoint_num >= in_endpoints_.size() ||
      in_endpoints_[endpoint_num] == nullptr) {
    AUSB_LOGW("received IN transfer complete event for unexpected endpoint %u",
              endpoint_num);
    return;
  }
  in_endpoints_[endpoint_num]->on_in_xfer_complete();
}

void EndpointManager::on_in_xfer_failed(uint8_t endpoint_num,
                                        XferFailReason reason) {
  if (endpoint_num >= in_endpoints_.size() ||
      in_endpoints_[endpoint_num] == nullptr) {
    AUSB_LOGW("received IN transfer failed event for unexpected endpoint %u",
              endpoint_num);
    return;
  }
  in_endpoints_[endpoint_num]->on_in_xfer_failed(reason);
}

void EndpointManager::on_out_xfer_complete(uint8_t endpoint_num,
                                           uint32_t bytes_read) {
  if (endpoint_num >= out_endpoints_.size() ||
      out_endpoints_[endpoint_num] == nullptr) {
    AUSB_LOGW("received OUT transfer complete event for unexpected endpoint %u",
              endpoint_num);
    return;
  }
  out_endpoints_[endpoint_num]->on_out_xfer_complete(bytes_read);
}

void EndpointManager::on_out_xfer_failed(uint8_t endpoint_num,
                                         XferFailReason reason) {
  if (endpoint_num >= out_endpoints_.size() ||
      out_endpoints_[endpoint_num] == nullptr) {
    AUSB_LOGW("received OUT transfer failed event for unexpected endpoint %u",
              endpoint_num);
    return;
  }
  out_endpoints_[endpoint_num]->on_out_xfer_failed(reason);
}

void EndpointManager::set_address(uint8_t address) {
  state_ = DeviceState::Address;
  hw_->set_address(address);
}

void EndpointManager::set_address_early(uint8_t address) {
  hw_->set_address_early(address);
}

Interface *EndpointManager::get_interface(uint8_t number) {
  if (number >= interfaces_.size()) {
    return nullptr;
  }
  return interfaces_[number];
}

void EndpointManager::set_configured(uint8_t config_id,
                                     asel::range<Interface *const> interfaces) {
  // The device state should be Address.  If SET_CONFIGURATION was called when
  // the state was already Configured, the handler should call unconfigure() as
  // the first step of their processing to close existing endpoints and
  // interfaces.
  assert(state_ == DeviceState::Address);
  // If SET_CONFIGURATION is called with a config ID of 0, only unconfigure()
  // should be called.
  assert(config_id != 0);

  assert(interfaces.size() < interfaces_.size());
  for (size_t n = 0; n < interfaces_.size(); ++n) {
    Interface *new_intf;
    if (n < interfaces.size()) {
      new_intf = interfaces[n];
      assert(new_intf != nullptr);
    } else {
      new_intf = nullptr;
    }
    interfaces_[n] = new_intf;
  }

  state_ = DeviceState::Configured;
  config_id_ = config_id;
}

void EndpointManager::unconfigure() {
  AUSB_LOGI("EndpointManager::unconfigure() invoked");

  unconfigure_endpoints_and_interfaces(XferFailReason::ConfigChanged,
                                       /*include_ep0=*/false);
  state_ = DeviceState::Address;
}

void EndpointManager::unconfigure_endpoints_and_interfaces(
    XferFailReason reason, bool include_ep0) {
  if (include_ep0) {
    in_endpoints_[0]->on_in_ep_unconfigured(reason);
    out_endpoints_[0]->on_out_ep_unconfigured(reason);
  }
  for (size_t n = 1; n < in_endpoints_.size(); ++n) {
    if (in_endpoints_[n] != nullptr) {
      in_endpoints_[n]->on_in_ep_unconfigured(reason);
    }
    in_endpoints_[n] = nullptr;
  }
  for (size_t n = 1; n < out_endpoints_.size(); ++n) {
    if (out_endpoints_[n] != nullptr) {
      out_endpoints_[n]->on_out_ep_unconfigured(reason);
    }
    out_endpoints_[n] = nullptr;
  }

  // Inform all interfaces that they have been unconfigured
  for (size_t n = 0; n < interfaces_.size(); ++n) {
    if (interfaces_[n] != nullptr) {
      interfaces_[n]->unconfigure();
    }
    interfaces_[n] = nullptr;
  }

  config_id_ = 0;
}

bool EndpointManager::open_in_endpoint(uint8_t endpoint_num,
                                       InEndpoint *endpoint,
                                       EndpointType type,
                                       uint16_t max_packet_size) {
  if (state_ == DeviceState::Default) {
    // We shouldn't be attempting to open endpoints until SET_ADDRESS and
    // then SET_CONFIGURATION has been called.
    return false;
  }

  if (endpoint_num >= in_endpoints_.size()) {
    AUSB_LOGE("cannot open IN endpoint %d: endpoint number is too large",
              endpoint_num);
    return false;
  }
  if (in_endpoints_[endpoint_num] != nullptr) {
    AUSB_LOGE(
        "cannot open IN endpoint %d: this endpoint number is already in use",
        endpoint_num);
    return false;
  }

  if (!hw_->open_in_endpoint(endpoint_num, type, max_packet_size)) {
    return false;
  }
  in_endpoints_[endpoint_num] = endpoint;
  return true;
}

bool EndpointManager::open_out_endpoint(uint8_t endpoint_num,
                                        OutEndpoint *endpoint,
                                        EndpointType type,
                                        uint16_t max_packet_size) {
  if (state_ == DeviceState::Default) {
    return false;
  }

  if (endpoint_num >= out_endpoints_.size()) {
    AUSB_LOGE("cannot open OUT endpoint %d: endpoint number is too large",
              endpoint_num);
    return false;
  }
  if (out_endpoints_[endpoint_num] != nullptr) {
    AUSB_LOGE(
        "cannot open OUT endpoint %d: this endpoint number is already in use",
        endpoint_num);
    return false;
  }

  if (!hw_->open_out_endpoint(endpoint_num, type, max_packet_size)) {
    return false;
  }
  out_endpoints_[endpoint_num] = endpoint;
  return true;
}

void EndpointManager::start_ctrl_in_write(MessagePipe *pipe,
                                          const void *data,
                                          uint32_t size) {
  auto status = hw_->start_write(pipe->endpoint_num(), data, size);
  if (status != XferStartResult::Ok) {
    AUSB_LOGE("error starting control IN transfer: %d",
              static_cast<int>(status));
    pipe->on_in_xfer_failed(XferFailReason::SoftwareError);
    return;
  }
}

void EndpointManager::start_in_write(uint8_t endpoint_num,
                                     const void *data,
                                     uint32_t size) {
  auto endpoint = in_endpoints_[endpoint_num];
  if (!endpoint) {
    AUSB_LOGE("attempted to start IN transfer on unopened endpoint %u",
              endpoint_num);
    return;
  }

  auto status = hw_->start_write(endpoint_num, data, size);
  if (status != XferStartResult::Ok) {
    AUSB_LOGE("error starting IN transfer: %d", static_cast<int>(status));
    endpoint->on_in_xfer_failed(XferFailReason::SoftwareError);
    return;
  }
}

void EndpointManager::start_ctrl_in_ack(MessagePipe *pipe) {
  auto status = hw_->start_read(pipe->endpoint_num(), nullptr, 0);
  if (status != XferStartResult::Ok) {
    AUSB_LOGE("error starting receipt of control IN ACK: %d",
              static_cast<int>(status));
    pipe->on_out_xfer_failed(XferFailReason::SoftwareError);
    return;
  }
}

void EndpointManager::start_ctrl_out_read(MessagePipe *pipe,
                                          void *data,
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
