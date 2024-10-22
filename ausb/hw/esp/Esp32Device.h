// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/SetupPacket.h"
#include "ausb/ausb_config.h"
#include "ausb/ausb_types.h"
#include "ausb/hw/HWDeviceBase.h"
#include "ausb/usb_types.h"

#include <esp_err.h>
#include <esp_private/usb_phy.h>
#include <hal/gpio_types.h>
#include <soc/usb_struct.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <system_error>

namespace ausb {

namespace device {
class EndpointManager;
}

enum class EspPhyType {
  Internal,
  External,
};

class EspTaskLoop;

/**
 * A USB device implementation for ESP32-S2 and ESP32-S3 chips.
 *
 * Note that Espressif provides rather limited documentation of their USB
 * core.  The ESP32-S3 Technical Reference Manual contains a high level
 * overview, but notes that full register documentation is subject to an NDA.
 *
 * However, the ESP32 USB core appears to be based on IP from Synopsys, and
 * several other microcontrollers also use this USB core IP.  For instance, the
 * STM32F429 chip from STMicroelectronics uses this core, and it's reference
 * manual (RM0090) provides much better documentation of the registers and
 * behavior.  Some details like FIFO sizing may be different between the chips,
 * but the documentation largely seems applicable:
 * https://www.st.com/en/microcontrollers-microprocessors/stm32f429-439/documentation.html
 *
 *
 * Synchronization behavior
 * ------------------------
 * The Esp32Device class is not thread safe.  Callers should ensure that it is
 * only accessed from a single task, or perform their own locking around calls
 * to Esp32Device functions.
 *
 * Esp32Device does internally manage synchronization done on the normal USB
 * task vs work done in interrupt handlers.
 *
 *
 * Some notes on implementation:
 * - We attempt to do as little work as possible in interrupt context,
 *   and do most work in the main USB task.  Copying data to/from the hardware
 *   FIFOs happens in the main USB task.  Interrupt handlers only signal the
 *   USB task when I/O is possible.
 */
class Esp32Device : public HWDeviceBase {
public:
  explicit constexpr Esp32Device(usb_dev_t *usb = &USB0) noexcept : usb_{usb} {}
  ~Esp32Device();

  /**
   * Initialize the USB device.
   *
   * If the device is self-powered, the vbus_monitor parameter should be set
   * to a GPIO that is monitoring whether the voltage on the VBUS line is
   * present or not.  This allows the device to tell whether the bus is
   * connected or not.  Note that ESP32 pins are not 5V tolerant, so a voltage
   * divider must be used.  For bus-powered devices, the vbus_monitor parameter
   * may be set to std::nullopt.
   */
  [[nodiscard]] std::error_code
  init(device::EndpointManager *mgr,
       EspTaskLoop *loop,
       EspPhyType phy_type = EspPhyType::Internal,
       std::optional<gpio_num_t> vbus_monitor = std::nullopt);

  /**
   * Reset and de-initialize the USB device.
   */
  void reset();

  /**
   * Process USB events that have been recorded by the interrupt handler.
   *
   * This should be called in the main USB task after it has been woken by the
   * interrupt handler.
   *
   * Returns false if there was no work ready to do, or true if it processed
   * at least one event generated by the interrupt handler.
   */
  bool process_events();

  /**
   * set_address() will be called after the status phase of a SET_ADDRESS
   * control transfer completes.
   */
  void set_address(uint8_t address);

  /**
   * set_address_early() will be called during the normal procesing phase of a
   * SET_ADDRESS transfer.
   */
  void set_address_early(uint8_t address);

  /**
   * Configure endpoint 0.
   *
   * This should be called in response to a BusEnumDone event.
   *
   * It is the caller's responsibility to ensure that the maximum packet size
   * is valid for the USB speed that was negotiated in the BusEnumDone event.
   * (MPS must be 8 for low speed and 64 for full speed.)
   *
   * Returns true on success, or false if called in an invalid state or with
   * bad arguments.
   */
  bool configure_ep0(uint8_t max_packet_size);

  [[nodiscard]] bool open_in_endpoint(uint8_t endpoint_num,
                                      EndpointType type,
                                      uint16_t max_packet_size);
  [[nodiscard]] bool open_out_endpoint(uint8_t endpoint_num,
                                       EndpointType type,
                                       uint16_t max_packet_size);

  /**
   * Configure a control endpoint to respond with a STALL error to IN and OUT
   * tokens.
   *
   * The stall state will automatically be cleared the next time we receive a
   * SETUP token, which starts a new control transfer.
   */
  void stall_control_endpoint(uint8_t endpoint_num);

  /**
   * Configure an OUT endpoint to respond to the next token with a STALL error.
   *
   * This method should not be used for endpoint 0.  Use stall_ep0() for
   * stalling the control endpoint.
   */
  void stall_out_endpoint(uint8_t endpoint_num);

  /**
   * Configure an IN endpoint to respond to the next token with a STALL error.
   *
   * This method should not be used for endpoint 0.  Use stall_ep0() for
   * stalling the control endpoint.
   */
  void stall_in_endpoint(uint8_t endpoint_num);

  /**
   * Flush the transmit FIFO for an IN endpoint.
   */
  void flush_tx_fifo(uint8_t endpoint_num);

  struct TxPacket {
    const void *data = nullptr;
    uint16_t size = 0;
  };

  /**
   * Write data to an IN endpoint.
   *
   * An WriteComplete event will be generated when the write has finished.
   * The caller must ensure the data buffer is available until the write is
   * finished.  stall_in_endpoint() may be called to abort the write operation
   * early if an error occurs.
   *
   * - Only one write may be in progress at a time for a given endpoint.
   *   XferStartResult::Busy will be returned if there is already a write in
   *   progress for this endpoint.
   *
   * - The size argument may be 0 to transmit a 0-length packet to the host.
   *   If the size is 0, the data argument may be null.
   *
   * - If the size is larger than the endpoint maximum packet size, the data
   *   will be sent in multiple packets, and all but the last will be the
   *   maximum packet size for the endpoint.  The final packet may be shorter
   *   than the maximum packet size if the data size is not a multiple of the
   *   endpoint MPS.  The WriteComplete event will not be generated until all
   *   data has been transmitted.
   *
   * - Users can split a single USB transfer into multiple start_write() calls
   *   if desired.  In this case it is the caller's responsibility to ensure
   *   that all start_write() calls except the final one use a size that is a
   *   multiple of the endpoint's maximum packet size.
   *
   * The device will signal the end of the write transfer with one of the
   * following events:
   * - InXferCompleteEvent
   * - InXferFailedEvent
   * - BusResetEvent (aborts all transfers on all endpoints)
   */
  [[nodiscard]] XferStartResult
  start_write(uint8_t endpoint, const void *data, uint32_t size);

  /**
   * Start reading data from an OUT endpoint.
   *
   * This will cause the device to begin accepting OUT packets from the host.
   *
   * The transfer will complete when:
   * - The host has transmitted exactly the amount of bytes requested in the
   *   size argument.
   * - The host transmits a smaller than maximum-packet-size packet for this
   *   endpoint (indicating the end of a transfer).
   * - The bus is reset.
   *
   * Notes regarding the size argument:
   * - If the size argument is not a multiple of the maximum packet size,
   *   we will still attempt to receive a final OUT packet even when the buffer
   *   does not contain room for a full OUT packet from the host.  If the host
   *   sends more data in the final packet than fits in the buffer, a
   *   BufferOverrun error will be returned to the application and the extra
   *   unexpected data sent by the host will be discarded.
   *
   *   This behavior can be useful if the application knows ahead of time how
   *   much data it expects the host to transfer, and has a buffer available
   *   with exactly that size.  If it will be an error for the host to send
   *   more data than expected, the application can provide only the buffer
   *   space required, and then handle the BufferOverrun error just as it would
   *   if process bad data contents received from the host.
   *
   *   If you wish to avoid BufferOverrun errors, always provide a size
   *   argument that is a multiple of the endpoint's maximum packet size.
   *
   * - If the size argument is 0, we will attempt to receive a single OUT
   *   packet that is expected to be 0 length.  If the host sends a non-zero
   *   length OUT packet, a BufferOverrun error will be generated.
   *
   * The device will signal the end of the write transfer with one of the
   * following events:
   * - OutXferCompleteEvent
   * - OutXferFailedEvent
   * - BusResetEvent (aborts all transfers on all endpoints)
   */
  [[nodiscard]] XferStartResult
  start_read(uint8_t endpoint, void *data, uint32_t size);

  /**
   * Acknowledge a control IN transfer on endpoint 0, by ACK'ing the next
   * 0-length OUT token from the host.
   */
  [[nodiscard]] XferStartResult ack_ctrl_in();

  /*
   * FIFO size configuration:
   * - There are 1024 bytes total available for all FIFOs.
   * - There is 1 RX FIFO shared by all endpoints, and 5 TX FIFOs.
   * - TX FIFO 0 is always assigned to endpoint 0.  The other 4 TX FIFOs are
   *   for use by other IN endpoints.  (Apart from endpoint 0, there can be at
   *   most 4 IN endpoints open at any one time, since each IN endpoint needs a
   *   dedicated TX FIFO.)
   *
   * Esp32Device::init() will set reasonable default FIFO sizes, so users don't
   * need to manually alter FIFO size settings unless they want to tweak
   * performance for their device configuration.
   *
   * The FIFO sizes should only be adjusted when the FIFOs are not currently in
   * use.  For the RX FIFO and TX FIFO 0, this means it should only be adjusted
   * after the bus is reset and before the call to configure_ep0().
   *
   * It is the user's responsibility to ensure that the FIFO sizes and
   * addresses chosen do not overlap.  If the start address for a FIFO is not
   * explicitly specified, the end address for the previous FIFO will be used.
   *
   * All FIFO sizes must be a multiple of 4 bytes.  TX FIFOs must be at least
   * as large as the max packet size of the endpoint they are assigned to (and
   * should ideally be a multiple of the max packet size, to avoid wasted
   * space).
   */

  /**
   * Configure the RX FIFO size.
   *
   * The RX FIFO is shared by all OUT endpoints.
   *
   * This method should only be called when the device is reset, before
   * configure_ep0() has been called.
   *
   * The RX FIFO size is recommended to include space for all of the following:
   * - At least 40 bytes for storing SETUP packets
   * - At least 4 bytes for Global OUT NAK state
   * - At least 4 bytes for each OUT endpoint (including EP0) for transfer
   *   complete status information
   * - 2 * (max_packet_size + 4), for the largest max packet size across all
   *   configured OUT endpoints.
   *
   * The RX FIFO always starts at the very beginning of the FIFO SPRAM.
   */
  [[nodiscard]] esp_err_t configure_rx_fifo(uint16_t size);

  /**
   * Configure the size of the TX FIFO for endpoint 0.
   *
   * This method should only be called when the device is reset, before
   * configure_ep0() has been called.  The FIFO size must be a multiple of 4,
   * and at least as large as endpoint 0's maximum packet size.
   *
   * If start_offset is 0, the TX FIFO will be configured to start at the end
   * of the RX FIFO.
   */
  [[nodiscard]] esp_err_t configure_tx_fifo0(uint16_t size,
                                             uint16_t start_offset = 0);

  /**
   * Configure the size of the TX FIFO N.
   *
   * If endpoint_num is 0, this FIFO will be automatically allocated to an
   * endpoint as needed when open_in_endpoint() is called.  If endpoint_num is
   * non-zero, this will mark this FIFO as only being usable for the specified
   * endpoint.  When that endpoint is opened it will be assigned to use this
   * FIFO.  An error will be returned when opening the endpoint if the FIFO
   * size is smaller than the max packet size for the endpoint.
   *
   * If start_offset is 0, the TX FIFO will be configured to start at the end
   * of the previous TX FIFO.
   *
   * It is the caller's responsibility to ensure that the configured RX and TX
   * FIFOs do not overlap with one another.
   */
  [[nodiscard]] esp_err_t configure_tx_fifo1(uint16_t size,
                                             uint16_t endpoint_num = 0,
                                             uint16_t start_offset = 0);
  [[nodiscard]] esp_err_t configure_tx_fifo2(uint16_t size,
                                             uint16_t endpoint_num = 0,
                                             uint16_t start_offset = 0);
  [[nodiscard]] esp_err_t configure_tx_fifo3(uint16_t size,
                                             uint16_t endpoint_num = 0,
                                             uint16_t start_offset = 0);
  [[nodiscard]] esp_err_t configure_tx_fifo4(uint16_t size,
                                             uint16_t endpoint_num = 0,
                                             uint16_t start_offset = 0);

  /**
   * Return the current RX FIFO size, in bytes.
   */
  uint16_t get_rx_fifo_size() const;

  /**
   * Return the current size of the specified TX FIFO.
   */
  uint16_t get_tx_fifo0_size() const;
  uint16_t get_tx_fifo1_size() const;
  uint16_t get_tx_fifo2_size() const;
  uint16_t get_tx_fifo3_size() const;
  uint16_t get_tx_fifo4_size() const;
  /**
   * Return the start offset of the specified TX FIFO.
   */
  uint16_t get_tx_fifo0_start() const;
  uint16_t get_tx_fifo1_start() const;
  uint16_t get_tx_fifo2_start() const;
  uint16_t get_tx_fifo3_start() const;
  uint16_t get_tx_fifo4_start() const;

private:
  // Speed bits used by the dcfg and dsts registers.
  // These unfortunately are not defined in soc/usb_reg.h
  enum Speed : uint32_t {
    High30Mhz = 0,
    Full30Mhz = 1,
    Low6Mhz = 2,
    Full48Mhz = 3,
  };
  // Max Packet Size setting for endpoint 0 in USB_DOEPCTL0_REG (USB_MPS0)
  enum EP0MaxPktSize : uint32_t {
    Mps64 = 0,
    Mps32 = 1,
    Mps16 = 2,
    Mps8 = 3,
  };

  static constexpr uint16_t kFifoMaxAddress = 1024;

  enum class InEPStatus : uint8_t {
    Unconfigured,
    Idle,
    Busy,
  };
  enum class OutEPStatus : uint8_t {
    Unconfigured,
    Idle,
    Busy,
  };

  struct InTransfer {
    void unconfigure() {
      reset(InEPStatus::Unconfigured);
    }
    void reset() {
      reset(InEPStatus::Idle);
    }
    void reset(InEPStatus st) {
      status = st;
      data = nullptr;
      size = 0;
      cur_xfer_end = 0;
      cur_fifo_ptr = 0;
    }
    void start(const void *buf, uint16_t len) {
      assert(status == InEPStatus::Idle);
      status = InEPStatus::Busy;
      data = buf;
      size = len;
      cur_xfer_end = 0;
      cur_fifo_ptr = 0;
    }

    InEPStatus status = InEPStatus::Unconfigured;
    const void *data = nullptr;
    uint32_t size = 0;
    // How much data we have told the hardware to transfer so far (in the
    // USB_D_XFERSIZE field of the DIEPTSIZE register).
    // This is an offset from the start of data.
    uint32_t cur_xfer_end = 0;
    // How much data has been pushed into the TX FIFO so far.
    // This is an offset from the start of data.
    // This should always be smaller than cur_xfer_end, as we have to update
    // DIEPTSIZE before pushing data into the FIFO..
    uint32_t cur_fifo_ptr = 0;
  };

  struct OutTransfer {
    void unconfigure() {
      reset(OutEPStatus::Unconfigured);
    }
    void reset() {
      reset(OutEPStatus::Idle);
    }
    void reset(OutEPStatus st) {
      status = st;
      data = nullptr;
      capacity = 0;
      bytes_read = 0;
    }
    void start(void *buf, uint16_t len) {
      assert(status == OutEPStatus::Idle);
      status = OutEPStatus::Busy;
      data = buf;
      capacity = len;
      bytes_read = 0;
    }

    OutEPStatus status = OutEPStatus::Unconfigured;
    void *data = nullptr;
    uint32_t capacity = 0;
    // How much data has been read from the endpoint.
    // Note that bytes_read may exceed capacity if the capacity was not a
    // multiple of the max packet size, and the final packet sent by the host
    // contained more data than the buffer had capacity for.  We will advance
    // bytes_read to indicate this, even though not all data fit into the
    // buffer.
    uint32_t bytes_read = 0;
  };

  struct InEndpointInterrupt {
    InEndpointInterrupt(uint8_t epnum, uint32_t flags)
        : endpoint_num(epnum), diepint(flags) {}

    uint8_t endpoint_num = 0;
    uint32_t diepint = 0;
  };
  struct OutEndpointInterrupt {
    OutEndpointInterrupt(uint8_t epnum, uint32_t flags)
        : endpoint_num(epnum), doepint(flags) {}

    uint8_t endpoint_num = 0;
    uint32_t doepint = 0;
  };
  struct RxFifoNonEmpty {};

  enum InterruptFlags : uint8_t {
    Reset = 0x01,
    Suspend = 0x02,
    Resume = 0x04,
    EnumDone = 0x08,
    InEndpointIntr = 0x10,
    OutEndpointIntr = 0x20,
    RxFifo = 0x40,
  };
  struct InterruptEvents {
    void clear();

    std::atomic<uint8_t> flags = 0;

    // Note: the DIEPINT and DOEPINT registers are 32-bits wide, but the upper
    // 16 bits are unused and reserved, so we only store the lower 16 bits
    // here.

    // DIEPINT flags for each IN endpoint
    std::array<uint16_t, USB_IN_EP_NUM> in_endpoints;
    // DOEPINT flags for each IN endpoint
    std::array<uint16_t, USB_OUT_EP_NUM> out_endpoints;
  };

  Esp32Device(Esp32Device const &) = delete;
  Esp32Device &operator=(Esp32Device const &) = delete;

  [[nodiscard]] esp_err_t esp_init(EspPhyType phy_type,
                                   std::optional<gpio_num_t> vbus_monitor);
  [[nodiscard]] esp_err_t init_phy(EspPhyType phy_type,
                                   std::optional<gpio_num_t> vbus_monitor);
  [[nodiscard]] esp_err_t enable_interrupts();
  static void static_interrupt_handler(void *arg);

  // Configure all IN/OUT endpoints to NAK tokens from the host.
  // These methods do not wait for the NAK flags to take effect.
  void nak_all_out_endpoints();
  void nak_all_in_endpoints();

  // Methods invoked from interrupt context.
  // All interrupt context methods have names starting with intr_*()
  //
  // For the most part we attempt to do as little work as possible in interrupt
  // context.  The interrupt handler mostly just sends events to the main USB
  // task, which get processed when process_events() is called.  This makes
  // concurrency and synchronization much easier to reason about, since most
  // state is only manipulated by the USB task.
  void intr_main();
  void intr_bus_reset();
  void intr_out_endpoint_main(InterruptEvents &events);
  void intr_in_endpoint_main(InterruptEvents &events);
  void intr_out_endpoint(InterruptEvents &events, uint8_t endpoint_num);
  void intr_in_endpoint(InterruptEvents &events, uint8_t endpoint_num);

  void process_bus_reset();
  void reset_device_state();
  void process_in_ep_interrupts(InterruptEvents &events);
  void process_in_ep_interrupt(uint8_t endpoint_num, uint32_t diepint);
  void initiate_next_write_xfer(uint8_t endpoint_num);
  void write_to_fifo(uint8_t endpoint_num);
  void copy_pkt_to_fifo(uint8_t fifo_num, const void *data, uint16_t pkt_size);
  void initiate_next_read_xfer(uint8_t endpoint_num);
  void process_rx_fifo();
  void process_one_rx_entry(uint32_t ctrl_word);
  void receive_packet(uint8_t endpoint_num, uint16_t packet_size);

  void process_out_ep_interrupts(InterruptEvents &events);
  void process_out_ep_interrupt(uint8_t endpoint_num, uint32_t doepint);
  void process_out_xfer_complete(uint8_t endpoint_num);
  void process_setup_received(uint8_t endpoint_num);

  void flush_all_transfers_on_reset();
  void disable_all_out_endpoints();
  void disable_all_in_endpoints();
  void flush_rx_fifo_helper();

  [[nodiscard]] esp_err_t configure_tx_fifo(uint8_t fifo_num,
                                            uint16_t size,
                                            uint16_t endpoint_num,
                                            uint16_t start_offset);
  [[nodiscard]] uint8_t allocate_tx_fifo(uint8_t endpoint_num,
                                         uint16_t max_packet_size);
  uint16_t get_tx_fifo_size(uint8_t fifo_num) const;
  uint16_t get_tx_fifo_start(uint8_t fifo_num) const;

  static uint16_t get_max_in_pkt_size(uint8_t endpoint_num, uint32_t diepctl);
  static uint8_t get_ep0_max_packet_size(EP0MaxPktSize mps_bits);

  void process_events(InterruptEvents &events);

  device::EndpointManager *mgr_ = nullptr;
  EspTaskLoop *loop_ = nullptr;
  usb_dev_t *usb_ = nullptr;
  usb_phy_handle_t phy_ = nullptr;
  intr_handle_t interrupt_handle_ = nullptr;

  union {
    std::array<uint32_t, 2> u32;
    SetupPacket setup;
  } setup_packet_ = {};

  /**
   * Two entries containing interrupt events that the USB task needs to
   * process.  There are two entries so that the interrupt routines can write
   * to one while the USB task is reading from the other.
   */
  std::array<InterruptEvents, 2> interrupt_events_ = {};

  /**
   * A pointer indicating which interrupt_events_ entry the interrupt routines
   * should write to.  This is updated by the USB task once it has finished
   * reading an entry.
   *
   * We currently pin the USB task and the USB interrupts to the same core, so
   * they cannot run together.  Therefore this cannot be updated while an
   * interrupt handler is running.  If we ever allowed the USB task to run on
   * the other core, we would need some sort of spinlock/mutex to prevent the
   * USB task from updating this pointer while an interrupt handler is running.
   */
  std::atomic<uint8_t> intr_event_index_;

  std::array<InTransfer, USB_IN_EP_NUM> in_transfers_;
  std::array<OutTransfer, USB_OUT_EP_NUM> out_transfers_;

  // An array tracking FIFO to endpoint allocations.
  // The array starts at FIFO 1 (since FIFO 0 is always assigned to endpoint
  // 0).  The value in the array is 0 if the FIFO is not currently assigned, or
  // an endpoint number if it is assigned.
  std::array<uint8_t, 4> tx_fifo_allocations_ = {};
};

} // namespace ausb
