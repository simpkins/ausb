// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"
#include "ausb/dev/ControlEndpoint.h"
#include "ausb/dev/DeviceEvent.h"
#include "ausb/hw/HWDevice.h"

#include <cstdint>
#include <memory>
#include <system_error>
#include <vector>

namespace ausb::device {

class Interface;

/**
 * This class is the main gateway between the hardware-independent code and the
 * hardware-specific layer.
 *
 * This class keeps track of the HWDevice implementation, as well as the
 * endpoints and interfaces defined by the application, and passes events back
 * and forth between the two.
 */
class EndpointManager {
public:
  constexpr explicit EndpointManager(
      HWDevice *hw, ControlEndpointCallback *ep0_handler) noexcept
      : hw_(hw), ep0_(this, ep0_handler) {}

  /**
   * Initialize the USB device.
   *
   * This accepts any arguments accepted by the underlying hardware device's
   * init() call, and forwards those to the hardware.
   */
  template<typename... Args>
  std::error_code init(Args... args) {
    const auto err = pre_init();
    if (err) {
      return err;
    }
    return hw_->init(std::forward<Args>(args)...);
  }

  /**
   * Reset and de-initialize the USB device.
   */
  void reset();

  /**
   * Run the main USB task loop.
   *
   * This continuously waits for new events from the hardware, then processes
   * them with handle_event().  If you wish to implement your own custom loop
   * for any reason, you can wait for events yourself and call handle_event()
   * to perform the event processing.
   */
  void loop();

  /**
   * Handle an event generated by the device hardware (or possibly sent from
   * another task).
   */
  void handle_event(const DeviceEvent &event);

  DeviceState state() const { return state_; }
  bool is_suspended() const { return dev_state_is_suspended(state_); }

  /**
   * Set the device address.
   *
   * This should generally only be invoked when handling a SET_ADDRESS request
   * on endpoint 0.
   *
   * Note that SET_ADDRESS requests are a bit unusual, and the address change
   * should be applied only after the final status phase of the message
   * completes.  This method should be called only when
   * CtrlOutXfer::ack_complete() is invoked for the SET_ADDRESS request.
   * During the normal request processing stage (before sending an
   * acknowledgement), set_address_early() should be invoked.
   */
  void set_address(uint8_t address);

  /**
   * set_address_early() should be called during the normal request processing
   * stage of a SET_ADDRESS request.
   *
   * This does not apply the address change yet, but gives some hardware types
   * the ability to do work here if desired.
   */
  void set_address_early(uint8_t address);

  /**
   * add_interface() should be called to define a new interface.
   *
   * This should generally be called as part of processing a SET_CONFIGURATION
   * request.  set_configured() should be called after all interfaces and
   * endpoints have been configured.
   *
   * The EndpointManager stores a pointer to the Interface object, but does not
   * own it.  The caller is responsible for ensuring that the Interface object
   * lives for at least as long as the device is configured.
   *
   * Returns false if an interface with this number is already defined.
   * Returns true on success.
   */
  bool add_interface(uint8_t number, Interface* interface);

  Interface *get_interface(uint8_t number);

  /**
   * Mark the device as configured.
   *
   * This should generally only be invoked when handling a SET_CONFIGURATION
   * request on endpoint 0, after all of the endpoints for this configuration
   * have been opened.
   */
  void set_configured();

  /**
   * Unconfigure the device.
   *
   * This closes all endpoints and puts the device back in the Address state.
   *
   * This should normally be called when handling a SET_CONFIGURATION request
   * with a config ID of 0.
   */
  void unconfigure();

  /**
   * Configure a control endpoint to respond to any future IN or OUT tokens
   * with a STALL error.
   *
   * This stall state will automatically be cleared the next time a SETUP
   * packet is received from the host.
   *
   * This method should generally only be invoked by a ControlEndpoint.
   */
  void stall_control_endpoint(uint8_t endpoint);

  /**
   * Send data for the current control IN transfer.
   *
   * This method should only be invoked by a ControlEndpoint.
   * on_in_xfer_complete() or on_in_xfer_failed() will be called on the
   * ControlEndpoint when the write is complete.  Note that on_in_xfer_failed()
   * may be invoked before start_ctrl_in_write() returns if there is an error
   * starting the write operation.
   *
   * If size is larger than the endpoint's maximum packet size the data will be
   * sent using multiple packets.  If the data being sent is only a partial
   * response, and more IN data for this transfer will be sent later, size must
   * be a multiple of the endpoint's maximum packet size.  All but the last
   * packet in an IN transfer must be full, maximum sized packets
   */
  void start_ctrl_in_write(ControlEndpoint *endpoint, const void *data,
                           uint32_t size);

  /**
   * Begin waiting for the host to acknowledge the status phase of a control IN
   * transfer.
   *
   * This method should only be invoked by a ControlEndpoint.
   * on_out_xfer_complete() or on_out_xfer_failed() will be called on the
   * ControlEndpoint when the write is complete.  (The on_out_*() methods are
   * called because the status phase of an IN transfer is a single 0-length OUT
   * packet.)
   */
  void start_ctrl_in_ack(ControlEndpoint* endpoint);

  /**
   * Begin receiving data for the current control OUT transfer.
   *
   * This method should only be invoked by a ControlEndpoint.
   * on_out_xfer_complete() or on_out_xfer_failed() will be called on the
   * ControlEndpoint when the read is complete.
   *
   * on_out_xfer_complete() will be invoked when one of:
   * - exactly size bytes have been received from the host
   * - the host sent a less-than-maximum-size packet on the endpoint,
   *   indicating the end of the transfer.
   *
   * If size is not a multiple of the endpoint's maximum packet size, this
   * requests to read a short packet from the bus as the final packet of this
   * read operation.  However, the host may send an OUT data packet with more
   * data than was expected.  If this occurs the read operation will fail with
   * a BufferOverrun error.
   *
   * If you want to receive OUT transfer data in multiple smaller buffers, you
   * can use multiple separate start_ctrl_out_read() calls to receive the data
   * in separate chunks.  However, note that:
   * - on all but the last start_ctrl_out_read(), the read size must be a
   *   multiple of the endpoint's maximum packet size.
   * - Only a single read operation may be in progress at a time on an
   *   endpoint, so a new start_ctrl_out_read() cannot be started until the
   *   previous read has completed.
   */
  void start_ctrl_out_read(ControlEndpoint *endpoint, void *data,
                           uint32_t size);

  /**
   * Begin to successfully acknowledge the status phase of a control OUT
   * transfer.
   *
   * This method should only be invoked by a ControlEndpoint.
   * on_in_xfer_complete() or on_in_xfer_failed() will be called on the
   * ControlEndpoint when the acknowledgement is complete.  (The on_in_*()
   * methods are called because the status phase of an OUT transfer is a single
   * 0-length IN packet.)
   */
  void start_ctrl_out_ack(ControlEndpoint* endpoint);

private:
  EndpointManager(EndpointManager const &) = delete;
  EndpointManager &operator=(EndpointManager const &) = delete;

  std::error_code pre_init();

  void on_bus_reset();
  void on_suspend();
  void on_resume();
  void on_enum_done(UsbSpeed speed);
  void on_setup_received(const SetupPacketEvent &packet);
  void on_in_xfer_complete(uint8_t endpoint_num);
  void on_in_xfer_failed(uint8_t endpoint_num, XferFailReason reason);
  void on_out_xfer_complete(uint8_t endpoint_num, uint32_t bytes_read);
  void on_out_xfer_failed(uint8_t endpoint_num, XferFailReason reason);

  DeviceState state_ = DeviceState::Uninit;
  uint8_t config_id_ = 0;
  bool remote_wakeup_enabled_ = false;

  HWDevice* hw_ = nullptr;
  ControlEndpoint ep0_;

  std::vector<Interface*> interfaces_;
};

} // namespace ausb::device
