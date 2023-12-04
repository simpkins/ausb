// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/DeviceEvent.h"
#include "ausb/usb_types.h"
#include "ausb/ausb_types.h"

#include <esp_err.h>
#include <esp_private/usb_phy.h>
#include <hal/gpio_types.h>
#include <soc/usb_struct.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

namespace ausb {

enum class EspPhyType {
  Internal,
  External,
};

class RxBuffer;

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
class Esp32Device {
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
  [[nodiscard]] esp_err_t
  init(EspPhyType phy_type = EspPhyType::Internal,
       std::optional<gpio_num_t> vbus_monitor = std::nullopt);

  /**
   * Reset and de-initialize the USB device.
   */
  void reset();

  /**
   * Wait for a USB event.
   *
   * Returns the USB event, or UninitializedEvent if the timeout expires before
   * an event is ready.
   */
  DeviceEvent wait_for_event(std::chrono::milliseconds timeout);

  /**
   * Get the FreeRTOS event queue used to receive USB events.
   *
   * This can be used if the caller wishes to manually fetch events, rather
   * than using the wait_for_event() function.
   */
  QueueHandle_t event_queue() const { return event_queue_; }

  RxBuffer create_rx_buffer(uint8_t endpoint, uint16_t max_packet_size,
                            uint8_t num_packets);

  /**
   * Configure endpoint 0.
   *
   * This should be called in response to a BusEnumDone event.
   *
   * The Esp32Device will store a pointer to this RxBuffer, but does not own
   * it.  The caller is responsible for ensuring that the RxBuffer object is
   * valid until the Esp32Device object is destroyed, or until the next bus
   * reset event (either initiated by the host or locally with a reset() call).
   *
   * It is the caller's responsibility to set the maximum packet size correctly
   * in the RxBuffer.  The packet size must be valid for the USB speed that was
   * negotiated in the BusEnumDone event.  (MPS must be 8 for low speed and 64
   * for full speed.)
   *
   * Returns true on success, or false if called in an invalid state or with
   * bad arguments.
   */
  bool configure_ep0(RxBuffer* buffer);

  /**
   * Configure Endpoint 0 to STALL the next IN or OUT token it receives.
   */
  void stall_ep0();

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
      const void* data = nullptr;
      uint16_t size = 0;
  };

  /**
   * Write data to an IN endpoint.
   *
   * An WriteComplete event will be generated when the write has finished.
   * The caller must ensure the data buffer is available until the
   * WriteComplete event is generated.  stall_in_endpoint() may be called to
   * abort the write operation early if an error occurs.
   *
   * - Only one write may be in progress at a time for a given endpoint.
   *   TxStartResult::Busy will be returned if there is already a write in
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
   */
  [[nodiscard]] TxStartResult start_write(uint8_t endpoint, const void *data,
                                          uint32_t size);

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

  enum class InEPStatus : uint8_t {
    Unconfigured,
    Idle,
    Busy,
  };

  struct InTransfer {
    void reset() {
      status = InEPStatus::Unconfigured;
      max_packet_size = 0;
      data = nullptr;
      size = 0;
      cur_xfer_end = 0;
      cur_fifo_ptr = 0;
    }
    void start(const void* buf, uint16_t len) {
      assert(status == InEPStatus::Idle);
      status = InEPStatus::Busy;
      data = buf;
      size = len;
      cur_xfer_end = 0;
      cur_fifo_ptr = 0;
    }

    InEPStatus status = InEPStatus::Unconfigured;
    uint16_t max_packet_size = 0;
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

  using Esp32DeviceEvent =
      std::variant<NoEvent, BusResetEvent, SuspendEvent, ResumeEvent,
                   BusEnumDone, SetupPacket, InEndpointInterrupt,
                   OutEndpointInterrupt, RxFifoNonEmpty>;
  static_assert(std::is_trivially_copyable_v<Esp32DeviceEvent>,
                "Esp32DeviceEvent must be trivially copyable");

  Esp32Device(Esp32Device const &) = delete;
  Esp32Device &operator=(Esp32Device const &) = delete;

  void send_event_from_isr(const Esp32DeviceEvent& event);
  template <typename T, typename... Args>
  void send_event_from_isr(Args &&... args) {
    Esp32DeviceEvent e{std::in_place_type<T>, std::forward<Args>(args)...};
    send_event_from_isr(e);
  }

  [[nodiscard]] esp_err_t init_phy(EspPhyType phy_type,
                                   std::optional<gpio_num_t> vbus_monitor);
  void all_endpoints_nak();
  [[nodiscard]] esp_err_t enable_interrupts();
  static void static_interrupt_handler(void *arg);

  // Methods invoked from interrupt context
  void intr_main();
  void intr_bus_reset();
  void intr_enum_done();
  void intr_out_endpoint_main();
  void intr_in_endpoint_main();
  void intr_out_endpoint(uint8_t endpoint_num);
  void intr_in_endpoint(uint8_t endpoint_num);

  DeviceEvent preprocess_event(Esp32DeviceEvent event);
  void process_bus_reset();
  DeviceEvent process_in_ep_interrupt(uint8_t endpoint_num, uint32_t diepint);
  void initiate_next_write_xfer(uint8_t endpoint_num);
  void write_to_fifo(uint8_t endpoint_num);
  void copy_pkt_to_fifo(uint8_t fifo_num, const void *data, uint16_t pkt_size);
  DeviceEvent process_rx_fifo();
  DeviceEvent process_one_rx_entry(uint32_t ctrl_word);
  void receive_packet(uint8_t endpoint_num, uint16_t packet_size);

  DeviceEvent process_out_ep_interrupt(uint8_t endpoint_num, uint32_t doepint);

  static constexpr uint8_t kMaxEventQueueSize = 32;

  StaticQueue_t queue_storage_ = {};
  QueueHandle_t event_queue_ = nullptr;
  usb_dev_t *usb_ = nullptr;
  usb_phy_handle_t phy_ = nullptr;
  intr_handle_t interrupt_handle_ = nullptr;
  std::array<uint8_t, sizeof(Esp32DeviceEvent) *kMaxEventQueueSize>
      queue_buffer_ = {};

  union {
    std::array<uint32_t, 2> u32;
    SetupPacket setup;
  } setup_packet_ = {};

  Esp32DeviceEvent pending_event_ = NoEvent(NoEventReason::Timeout);

  // A pointer to the (software) RX buffer for the control endpoint.
  // Note that this object is not owned by us.  Our user is responsible
  // for ensuring it remains valid until the bus is reset.
  RxBuffer* ep0_rx_buffer_ = nullptr;

  std::array<InTransfer, USB_IN_EP_NUM> in_transfers_;
};

} // namespace ausb
