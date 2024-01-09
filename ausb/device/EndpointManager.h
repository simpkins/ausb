// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"
#include "ausb/device/EndpointZero.h"
#include "ausb/hw/HWDevice.h"

#include <asel/array.h>
#include <asel/range.h>
#include <asel/utility.h>

#include <cstdint>
#include <memory>
#include <system_error>
#include <vector>

namespace ausb::device {

class Interface;
class InEndpoint;
class OutEndpoint;

/**
 * This class is the main gateway between the hardware-independent code and the
 * hardware-specific layer.
 *
 * This class keeps track of the HWDevice implementation, as well as the
 * endpoints and interfaces defined by the application.  It is also the main
 * API that the HWDevice implementations use when informing the higher-level
 * code of hardware events that have occurred.
 */
class EndpointManager {
public:
  constexpr explicit EndpointManager(
      HWDevice *hw, EndpointZeroCallback *ep0_callback) noexcept
      : hw_(hw), ep0_(this, ep0_callback) {}

  /**
   * Initialize the USB device.
   *
   * This accepts any arguments accepted by the underlying hardware device's
   * init() call, and forwards those to the hardware.
   */
  template <typename... Args>
  std::error_code init(Args... args) {
    return hw_->init(this, asel::forward<Args>(args)...);
  }

  /**
   * Reset and de-initialize the USB device.
   */
  void reset();

  DeviceState state() const {
    return state_;
  }
  uint8_t config_id() const {
    return config_id_;
  }
  bool is_suspended() const {
    return dev_state_is_suspended(state_);
  }

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
   * Mark the device as configured.
   *
   * This should generally only be invoked when handling a SET_CONFIGURATION
   * request on endpoint 0, after all of the endpoints for this configuration
   * have been opened.
   *
   * The endpoints for this configuration should typically be opened before
   * calling set_configured().
   */
  void set_configured(uint8_t config_id,
                      asel::range<Interface *const> interfaces);
  template <size_t N>
  void set_configured(uint8_t config_id,
                      const asel::array<Interface *, N> &interfaces) {
    set_configured(config_id, asel::range<Interface *const>(interfaces));
  }
  template <typename... IntfT>
  void set_configured(uint8_t config_id,
                      Interface *intf1,
                      IntfT *...other_interfaces) {
    asel::array intf_array(
        asel::to_array<Interface *>({intf1, other_interfaces...}));
    set_configured(config_id, asel::range<Interface *const>(intf_array));
  }

  /**
   * Unconfigure the device.
   *
   * This closes all endpoints and puts the device back in the Address state.
   *
   * This should normally be called when handling a SET_CONFIGURATION request
   * with a config ID of 0.
   */
  void unconfigure();

  [[nodiscard]] bool open_in_endpoint(uint8_t endpoint_num,
                                      InEndpoint *endpoint,
                                      EndpointType type,
                                      uint16_t max_packet_size);
  [[nodiscard]] bool open_out_endpoint(uint8_t endpoint_num,
                                       OutEndpoint *endpoint,
                                       EndpointType type,
                                       uint16_t max_packet_size);

  /**
   * Get an interface by index.
   *
   * Returns the interface with this index, or nullptr if the index is larger
   * or equal to the number of currently configured interfaces.
   */
  [[nodiscard]] Interface *get_interface(uint8_t index);

  /**
   * Configure a message pipe to respond to any future IN or OUT tokens
   * with a STALL error.
   *
   * This configures both the IN and OUT endpoints associated with the pipe to
   * respond to tokens with a STALL error.  This stall state will automatically
   * be cleared the next time a SETUP packet is received from the host.
   *
   * This method should generally only be invoked by a MessagePipe.
   */
  void stall_message_pipe(uint8_t endpoint_num);

  /**
   * Send data for the current control IN transfer.
   *
   * This method should only be invoked by a MessagePipe.
   * on_in_xfer_complete() or on_in_xfer_failed() will be called on the
   * MessagePipe when the write is complete.  Note that on_in_xfer_failed() may
   * be invoked before start_ctrl_in_write() returns if there is an error
   * starting the write operation.
   *
   * If size is larger than the endpoint's maximum packet size the data will be
   * sent using multiple packets.  If the data being sent is only a partial
   * response, and more IN data for this transfer will be sent later, size must
   * be a multiple of the endpoint's maximum packet size.  All but the last
   * packet in an IN transfer must be full, maximum sized packets
   */
  void start_ctrl_in_write(MessagePipe *pipe, const void *data, uint32_t size);

  /**
   * Begin waiting for the host to acknowledge the status phase of a control IN
   * transfer.
   *
   * This method should only be invoked by a MessagePipe.
   * on_out_xfer_complete() or on_out_xfer_failed() will be called on the
   * MessagePipe when the write is complete.  (The on_out_*() methods are
   * called because the status phase of an IN transfer is a single 0-length OUT
   * packet.)
   */
  void start_ctrl_in_ack(MessagePipe *pipe);

  void start_in_write(uint8_t endpoint_num, const void *data, uint32_t size);

  /**
   * Begin receiving data for the current control OUT transfer.
   *
   * This method should only be invoked by a MessagePipe.
   * on_out_xfer_complete() or on_out_xfer_failed() will be called on the
   * MessagePipe when the read is complete.
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
  void start_ctrl_out_read(MessagePipe *pipe, void *data, uint32_t size);

  /**
   * Begin to successfully acknowledge the status phase of a control OUT
   * transfer.
   *
   * This method should only be invoked by a MessagePipe.
   * on_in_xfer_complete() or on_in_xfer_failed() will be called on the
   * MessagePipe when the acknowledgement is complete.  (The on_in_*()
   * methods are called because the status phase of an OUT transfer is a single
   * 0-length IN packet.)
   */
  void start_ctrl_out_ack(MessagePipe *pipe);

  HWDevice *hw() {
    return hw_;
  }

  ///////////////////////////////////////////////////////////////////////////
  // The following methods should only be invoked by HWDevice implementations
  //
  // These must be invoked from the main USB task, and should never be invoked
  // from interrupt handlers.
  ///////////////////////////////////////////////////////////////////////////

  void on_bus_reset();
  void on_suspend();
  void on_resume();
  void on_enum_done(UsbSpeed speed);
  void on_setup_received(uint8_t endpoint_num, const SetupPacket &packet);
  void on_in_xfer_complete(uint8_t endpoint_num);
  void on_in_xfer_failed(uint8_t endpoint_num, XferFailReason reason);
  void on_out_xfer_complete(uint8_t endpoint_num, uint32_t bytes_read);
  void on_out_xfer_failed(uint8_t endpoint_num, XferFailReason reason);

private:
  EndpointManager(EndpointManager const &) = delete;
  EndpointManager &operator=(EndpointManager const &) = delete;

  std::error_code pre_init();

  void unconfigure_endpoints_and_interfaces();

  DeviceState state_ = DeviceState::Uninit;
  uint8_t config_id_ = 0;
  bool remote_wakeup_enabled_ = false;

  HWDevice *hw_ = nullptr;
  EndpointZero ep0_;

  // Array storing pointers to the currently configured interfaces.
  //
  // The set of interfaces is defined by the current configuration, and can
  // only be changed with a set_configured() or unconfigure() call.
  //
  // TODO: We could require that the user provide storage for this array,
  // where we just point to an array that they provide in set_configured().
  // However, this seems easy for users to accidentally provide an array with
  // an insufficient lifetime.  Perhaps define a Configuration class, which
  // contains an array of configured interfaces plus the config descriptor?
  // That would make it easier for callers to provide storage for this array.
  static constexpr size_t kMaxNumInterfaces = 6;
  asel::array<Interface *, kMaxNumInterfaces> interfaces_ = {};

  // Arrays storing points to the currently configured endpoints.
  //
  // The set of endpoints currently in use can be changed on the fly without
  // changing the overall device configuration: a SET_INTERFACE call may be
  // used to change an interface behavior, which can change the endpoints it
  // uses.
  //
  // We provide storage for these arrays primarily to make our APIs easier to
  // use.  This does place limits on the max number of endpoints, and result in
  // wasted space if fewer endpoints are in use.
  //
  // The endpoint number is encoded as 4 bits in USB token packets, so
  // the maximum possible endpoint number is 15, allowing for up to 16 IN
  // endpoints and 16 OUT endpoints (including endpoint 0).  In practice most
  // device hardware supports fewer endpoints than this.
  static constexpr size_t kMaxNumOutEndpoints = 6; // TODO: move to build config
  static constexpr size_t kMaxNumInEndpoints = 6;  // TODO: move to build config
  asel::array<InEndpoint *, kMaxNumInEndpoints> in_endpoints_ = {};
  asel::array<OutEndpoint *, kMaxNumOutEndpoints> out_endpoints_ = {};
};

} // namespace ausb::device
