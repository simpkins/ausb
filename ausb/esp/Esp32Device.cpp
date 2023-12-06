// Copyright (c) 2023, Adam Simpkins
#include "ausb/esp/Esp32Device.h"

#include "ausb/esp/EspError.h"

#include <esp_check.h>
#include <esp_log.h>
#include <hal/usb_hal.h>
#include <soc/usb_otg_periph.h>
#include <soc/usb_periph.h>
#include <soc/usb_pins.h>

#include <atomic>
#include <cinttypes>
#include <cstring>

#define AUSB_VERBOSE_LOGGING

// ESP_LOGx level aren't checked when using ESP_EARLY_LOGI() during
// interrupt context.  Provide our own log macro so we can have some
// debug log statements that are normally disabled but that can be enabled
// when desired for debugging and development.
#ifdef AUSB_VERBOSE_LOGGING
#define ISR_LOGD(arg, ...) ESP_EARLY_LOGI(LogTag, arg, ##__VA_ARGS__)
#define ISR_LOGV(arg, ...) ESP_EARLY_LOGI(LogTag, arg, ##__VA_ARGS__)
#else
#define ISR_LOGD(arg, ...) (static_cast<void>(0))
#define ISR_LOGV(arg, ...) (static_cast<void>(0))
#endif

namespace {
const char *LogTag = "ausb.esp32";

// These helper functions avoid compiler warnings about using |= or &= on
// volatile integers.
inline void set_bits(volatile uint32_t &value, uint32_t bits) {
  value = (value | bits);
}
inline void clear_bits(volatile uint32_t &value, uint32_t bits) {
  value = (value & ~bits);
}

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

// The definitions for some of the endpoint register fields appears to be
// incorrect in soc/usb_reg.h.  It looks like some of the settings were
// copy-and-pasted from endpoint 0 to all other endpoints, but the other
// endpoints should be using different values for several fields in DIEPCTL,
// DIEPTSIZ, DOEPCTL, and DOEPTSIZ.
// (https://github.com/espressif/esp-idf/issues/12710)
constexpr uint32_t Correct_USB_D_MPS1_V = 0x7FF;
constexpr uint32_t Correct_USB_D_MPS1_M =
    (Correct_USB_D_MPS1_V << USB_D_MPS1_S);
constexpr uint32_t Correct_USB_D_PKTCNT1_V = 0x3FF;
constexpr uint32_t Correct_USB_D_PKTCNT1_M =
    (Correct_USB_D_PKTCNT1_V << USB_D_PKTCNT1_S);
constexpr uint32_t Correct_USB_D_XFERSIZE1_V = 0x7FFFF;
constexpr uint32_t Correct_USB_D_XFERSIZE1_M =
    (Correct_USB_D_XFERSIZE1_V << USB_D_XFERSIZE1_S);

constexpr uint32_t Correct_USB_MPS1_V = 0x7FF;
constexpr uint32_t Correct_USB_MPS1_M =
    (Correct_USB_MPS1_V << USB_MPS1_S);
constexpr uint32_t Correct_USB_PKTCNT1_V = 0x3FF;
constexpr uint32_t Correct_USB_PKTCNT1_M =
    (Correct_USB_PKTCNT1_V << USB_PKTCNT1_S);
constexpr uint32_t Correct_USB_XFERSIZE1_V = 0x7FFFF;
constexpr uint32_t Correct_USB_XFERSIZE1_M =
    (Correct_USB_XFERSIZE1_V << USB_XFERSIZE1_S);

} // unnamed namespace

namespace ausb {

Esp32Device::~Esp32Device() {
  reset();
}

DeviceEvent Esp32Device::wait_for_event(std::chrono::milliseconds timeout) {
  // If pending_event_ is set, we have a previously-received interrupt event
  // that still needs additional processing to be done.
  if (!std::holds_alternative<NoEvent>(pending_event_)) {
    const auto evt = pending_event_;
    pending_event_ = NoEvent(NoEventReason::Timeout);
    ESP_LOGV(LogTag, "USB wait_for_event() processing existing pending event");
    // If process_event() returns NoEvent we could potentially fall through
    // and process an event from the queue.  However, for now it's simpler to
    // just return back to the caller and let them call wait_for_event() again
    // on the next loop.
    return process_event(evt);
  }

  TickType_t max_wait = pdMS_TO_TICKS(timeout.count());
  Esp32DeviceEvent event(NoEvent(NoEventReason::Timeout));
  if (xQueueReceive(event_queue_, &event, max_wait) == pdTRUE) {
    return process_event(event);
  } else {
    return NoEvent(NoEventReason::Timeout);
  }
}

DeviceEvent Esp32Device::process_event(Esp32DeviceEvent event) {
  return std::visit(
      overloaded{
          [this](const NoEvent &ev) -> DeviceEvent { return ev; },
          [this](const BusResetEvent &ev) -> DeviceEvent {
            process_bus_reset();
            return ev;
          },
          [this](const SuspendEvent &ev) -> DeviceEvent { return ev; },
          [this](const ResumeEvent &ev) -> DeviceEvent { return ev; },
          [this](const BusEnumDone &ev) -> DeviceEvent { return ev; },
          [this](const SetupPacketEvent &ev) -> DeviceEvent { return ev; },
          [this](const InEndpointInterrupt &ev) -> DeviceEvent {
            return process_in_ep_interrupt(ev.endpoint_num, ev.diepint);
          },
          [this](const OutEndpointInterrupt &ev) -> DeviceEvent {
            return process_out_ep_interrupt(ev.endpoint_num, ev.doepint);
          },
          [this](const RxFifoNonEmpty &) -> DeviceEvent {
            return process_rx_fifo();
          }},
      event);
}

void Esp32Device::process_bus_reset() {
  for (size_t endpoint_num = 0; endpoint_num < in_transfers_.size();
       ++endpoint_num) {
    in_transfers_[endpoint_num].unconfigure();
  }
  for (size_t endpoint_num = 0; endpoint_num < out_transfers_.size();
       ++endpoint_num) {
    out_transfers_[endpoint_num].unconfigure();
  }
}

// Note that process_in_ep_interrupt() runs in the main USB task, and not in an
// interrupt context.  The interrupt handler simply sends the event to the main
// USB task, and this function handles it on the USB task.
DeviceEvent Esp32Device::process_in_ep_interrupt(uint8_t endpoint_num,
                                                 uint32_t diepint) {
  auto &xfer = in_transfers_[endpoint_num];
  if (xfer.status != InEPStatus::Busy) {
    // It's possible that this could happen if we recently processed
    // application-initiated request to abort the transfer and this raced with
    // us receiving the interrupt.
    ESP_LOGD(LogTag,
             "IN interrupt on endpoint %d, but transfer was no "
             "longer in progress",
             endpoint_num);
    return NoEvent(NoEventReason::HwProcessing);
  }

  // Transfer timeout
  // This is only sent for control IN endpoints.
  if (diepint & USB_D_TIMEOUT0_M) {
    ESP_LOGW(LogTag, "USB IN transfer timeout on EP%u", endpoint_num);
    flush_tx_fifo(0);
    xfer.reset();
    return InXferFailedEvent(endpoint_num, XferFailReason::Timeout);
  }

  // IN transfer complete
  // Note that this means a hardware transfer was complete.  The full transfer
  // requested by the application may not yet be complete if it could not fit
  // into a single HW transfer.
  if (diepint & USB_D_XFERCOMPL0_M) {
    ESP_LOGD(LogTag, "USB: IN transfer complete on EP%u", endpoint_num);
    if (xfer.cur_xfer_end == xfer.size) {
      xfer.reset();
      return InXferCompleteEvent(endpoint_num);
    } else {
      initiate_next_write_xfer(endpoint_num);
      return NoEvent(NoEventReason::HwProcessing);
    }
  }

  // FIFO empty
  // Note that this flag will usually also be set when USB_D_XFERCOMPL0_M
  // is set.  However, we only care about processing this event when
  // USB_D_XFERCOMPL0_M is not set.
  if (diepint & USB_D_TXFEMP0_M) {
    ESP_LOGD(LogTag, "TX FIFO space available on endpoint %d", endpoint_num);
    write_to_fifo(endpoint_num);
    return NoEvent(NoEventReason::HwProcessing);
  }

  // It's sort of unexpected to reach here with no flags set.
  ESP_LOGD(LogTag, "IN interrupt on endpoint %d, but no flags set",
           endpoint_num);
  return NoEvent(NoEventReason::HwProcessing);
}

DeviceEvent Esp32Device::process_out_ep_interrupt(uint8_t endpoint_num,
                                                  uint32_t doepint) {
  // If the XFERCOMPL and SETUP flags are both set, the most likely scenario is
  // that we just finished an OUT transfer that was the status phase of a
  // control IN transfer, and then we received a new SETUP packet to start a
  // new transfer.
  //
  // Process the XFERCOMPL first, since we generally want to inform the caller
  // of the previous transfer finishing before we tell them about the new SETUP
  // packet.
  if (doepint & USB_XFERCOMPL0_M) {
    ESP_LOGD(LogTag, "EP%u OUT: transfer complete", endpoint_num);
    auto event = process_out_xfer_complete(endpoint_num);
    if (!std::holds_alternative<NoEvent>(event)) {
      // If USB_SETUP0_M was also set, remember that event so we can process it
      // the next time wait_for_event() is called.
      if (doepint & USB_SETUP0_M) {
        pending_event_ =
            OutEndpointInterrupt(endpoint_num, doepint & ~USB_XFERCOMPL0_M);
      }
      return event;
    }
  }

  // Setup packet receipt done
  //
  // The hardware generates this event after we pop the SetupComplete event
  // from the RX FIFO.
  //
  // Note that when this interrupt occurs the hardware will reset the packet
  // count and byte size to 0 in the doeptsiz register, and will clear the
  // endpoint enable flag in doepctl.  These always need to be explicitly
  // updated to enable OUT transfer receipt after a SETUP packet.
  if ((doepint & USB_SETUP0_M)) {
    ESP_LOGD(LogTag, "EP%u OUT: SETUP receive complete", endpoint_num);

    // If XFERCOMPL was also set (should be unlikely?), add an event back to
    // the front of the queue to process this flag the next time
    // wait_for_event() is called.
    if (doepint & USB_XFERCOMPL0_M) {
      pending_event_ =
          OutEndpointInterrupt(endpoint_num, doepint & ~USB_SETUP0_M);
    }

    // Inform the application of the SETUP packet
    return SetupPacketEvent(endpoint_num, setup_packet_.setup);
  }

  return NoEvent(NoEventReason::HwProcessing);
}

DeviceEvent Esp32Device::process_out_xfer_complete(uint8_t endpoint_num) {
  auto &xfer = out_transfers_[endpoint_num];
  if (xfer.status != OutEPStatus::Busy) {
    // It's possible that this could happen if we recently processed
    // application-initiated request to abort the transfer and this raced
    // with us receiving the interrupt.
    ESP_LOGD(LogTag,
             "OUT xfer complete interrupt on endpoint %d, but transfer "
             "was no longer in progress",
             endpoint_num);
    // Fall through to continue processing other interrupt flags
    return NoEvent(NoEventReason::HwProcessing);
  }

  // If we the transfer ended before we read all requested data,
  // this means the host sent a short byte, which ends the transfer.
  // Otherwise, we received all data requested, and if the software-initiated
  // transfer still wants more data, then start a new hardware transfer.
  const auto doeptsiz = usb_->out_ep_reg[endpoint_num].doeptsiz;
  const uint32_t pkts_left = (doeptsiz >> USB_PKTCNT0_S) & USB_PKTCNT0_V;
  const uint32_t bytes_left = (doeptsiz >> USB_XFERSIZE0_S) & USB_XFERSIZE0_V;
  if (pkts_left == 0 && bytes_left == 0 && xfer.bytes_read < xfer.capacity) {
    // Start the next HW transfer
    initiate_next_read_xfer(endpoint_num);
    return NoEvent(NoEventReason::HwProcessing);
  }

  // The overall software transfer is now complete
  if (xfer.bytes_read > xfer.capacity) {
    // Buffer overrun.  The software asked for a partial packet amount, and
    // the host sent more than was expected.
    return OutXferFailedEvent(endpoint_num, XferFailReason::BufferOverrun);
  }

  return OutXferCompleteEvent(endpoint_num, xfer.bytes_read);
}

void Esp32Device::send_event_from_isr(const Esp32DeviceEvent& event) {
  BaseType_t higher_prio_task_woken;
  BaseType_t res =
      xQueueSendToBackFromISR(event_queue_, &event, &higher_prio_task_woken);
  portYIELD_FROM_ISR(higher_prio_task_woken);

  if (res != pdPASS) {
    ESP_EARLY_LOGE(LogTag, "USB event queue full; unable to send event");
    // If a USB event is lost we can't necessarily recover from this, as our
    // state may become inconsistent.  Abort for now.
    //
    // TODO: We perhaps could keep local storage space for 1 event, then store
    // the event there and disable interrupts until the queue has been read
    // from.  The main task can then re-enable interrupts once it has freed
    // space in the queue.
    abort();
  }
}

std::error_code Esp32Device::init(EspPhyType phy_type,
                            std::optional<gpio_num_t> vbus_monitor) {
  const auto esp_err = esp_init(phy_type, vbus_monitor);
  return make_esp_error(esp_err);
}

esp_err_t Esp32Device::esp_init(EspPhyType phy_type,
                            std::optional<gpio_num_t> vbus_monitor) {
  ESP_LOGI(LogTag, "USB device init");

  event_queue_ = xQueueCreateStatic(kMaxEventQueueSize, sizeof(DeviceEvent),
                                    queue_buffer_.data(), &queue_storage_);

  const auto err = init_phy(phy_type, vbus_monitor);
  if (err != ESP_OK) {
    return err;
  }

  // Ensure the data line pull-up is disabled as we perform initialization.
  set_bits(usb_->dctl, USB_SFTDISCON_M);

  // During initialization, send a stall on receipt of
  // any non-zero length OUT packet.  Also set speed.
  set_bits(usb_->dcfg, USB_NZSTSOUTHSHK_M | Speed::Full48Mhz);

  // Configure AHB interrupts:
  // - Global interrupts enable flag
  // - Enable periodic TxFIFO empty interrupt (recommended by STM32 reference
  //   manual)
  // - Configure the non-periodic TxFIFO empty interrupt to fire when
  //   completely empty rather than half empty
  set_bits(usb_->gahbcfg,
           USB_GLBLLNTRMSK_M | USB_PTXFEMPLVL_M | USB_NPTXFEMPLVL_M);

  // USB configuration:
  auto gusbcfg = usb_->gusbcfg;
  // We currently don't support HNP or SRP
  // Clear the timeout calibration and turnaround time bits, as we will set
  // these fields below.
  gusbcfg &= ~(USB_HNPCAP_M | USB_SRPCAP_M | USB_TOUTCAL_M | USB_USBTRDTIM_M);
  // Force device mode (rather than OTG)
  gusbcfg |= USB_FORCEDEVMODE_M;
  // Set the timeout calibration to the maximum value
  gusbcfg |= USB_TOUTCAL_M;
  // Set the USB turnaround time to 5 (recommended default value from the docs)
  gusbcfg |= (5 << USB_USBTRDTIM_S);
  usb_->gusbcfg = gusbcfg;

  // OTG configuration:
  // - disable overrides
  clear_bits(usb_->gotgctl,
             USB_BVALIDOVVAL_M | USB_BVALIDOVEN_M | USB_VBVALIDOVVAL_M);

  // Configure all endpoints to NAK
  all_endpoints_nak();

  // Flush all TX FIFOs, and wait for the flush to complete
  set_bits(usb_->grstctl, (0x10 << USB_TXFNUM_S) | USB_TXFFLSH_M);
  while (usb_->grstctl & USB_TXFFLSH_M) {
  }
  // Flush the RX FIFO, and wait for the flush to complete
  set_bits(usb_->grstctl, USB_RXFFLSH_M);
  while (usb_->grstctl & USB_RXFFLSH_M) {
  }

  // Interrupt configuration
  usb_->gintmsk = 0;          // mask all interrupts
  usb_->gotgint = 0xffffffff; // clear OTG interrupts
                              // (even though we leave OTG interrupts disabled)
  usb_->gintsts = 0xffffffff; // clear pending interrupts
  usb_->gintmsk = USB_MODEMISMSK_M | USB_OTGINTMSK_M | USB_RXFLVIMSK_M |
                  USB_ERLYSUSPMSK_M | USB_USBSUSPMSK_M | USB_USBRSTMSK_M |
                  USB_ENUMDONEMSK_M | USB_RESETDETMSK_M | USB_DISCONNINTMSK_M;

  ESP_RETURN_ON_ERROR(
      enable_interrupts(), LogTag, "error enabling USB interrupts");

  // Enable the data line pull-up to connect to the bus.
  clear_bits(usb_->dctl, USB_SFTDISCON_M);

  return ESP_OK;
}

void Esp32Device::reset() {
  usb_->gintmsk = 0; // Mask all interrupts
  usb_->daintmsk = 0;
  set_bits(usb_->dctl, USB_SFTDISCON_M); // Disable the data pull-up line

  // TODO: flush RX FIFO
  // TODO: flush all TX FIFOs
  pending_event_ = NoEvent(NoEventReason::Timeout);
  for (size_t endpoint_num = 0; endpoint_num < in_transfers_.size();
       ++endpoint_num) {
    in_transfers_[endpoint_num].reset();
  }
  for (size_t endpoint_num = 0; endpoint_num < out_transfers_.size();
       ++endpoint_num) {
    out_transfers_[endpoint_num].reset();
  }

  if (interrupt_handle_) {
    // esp_intr_free() will wait for any in-progress interrupt handlers to
    // finish before it returns.
    esp_intr_free(interrupt_handle_);
    usb_->gintmsk = 0;
    usb_->daintmsk = 0;
  }
  if (phy_) {
    usb_del_phy(phy_);
  }

  // No need to call vQueueDelete(event_queue_) here since event_queue_ was
  // allocated with xQueueCreateStatic() rather than xQueueCreate().
}

esp_err_t Esp32Device::init_phy(EspPhyType phy_type,
                                std::optional<gpio_num_t> vbus_monitor) {
  usb_phy_ext_io_conf_t ext_io_conf = {
      .vp_io_num = USBPHY_VP_NUM,
      .vm_io_num = USBPHY_VM_NUM,
      .rcv_io_num = USBPHY_RCV_NUM,
      .oen_io_num = USBPHY_OEN_NUM,
      .vpo_io_num = USBPHY_VPO_NUM,
      .vmo_io_num = USBPHY_VMO_NUM,
  };
  usb_phy_otg_io_conf_t self_powered_otg_io_conf =
      USB_PHY_SELF_POWERED_DEVICE(vbus_monitor ? *vbus_monitor : -1);
  usb_phy_config_t phy_conf = {
      .controller = USB_PHY_CTRL_OTG,
      .target = phy_type == EspPhyType::External ? USB_PHY_TARGET_EXT
                                                 : USB_PHY_TARGET_INT,
      .otg_mode = USB_OTG_MODE_DEVICE,
      .otg_speed = USB_PHY_SPEED_UNDEFINED,
      .ext_io_conf = phy_type == EspPhyType::External ? &ext_io_conf : nullptr,
      .otg_io_conf = vbus_monitor ? &self_powered_otg_io_conf : nullptr,
  };
  ESP_RETURN_ON_ERROR(
      usb_new_phy(&phy_conf, &phy_), LogTag, "error configuring USB PHY");
  return ESP_OK;
}

void Esp32Device::all_endpoints_nak() {
  // Set the NAK bit on all OUT endpoints, so they will NAK any OUT packets
  // sent by the host.
  for (int ep_num = 0; ep_num < USB_OUT_EP_NUM; ++ep_num) {
    set_bits(usb_->out_ep_reg[ep_num].doepctl, USB_DO_SNAK0_M);
  }
}

esp_err_t Esp32Device::enable_interrupts() {
  return esp_intr_alloc(ETS_USB_INTR_SOURCE, ESP_INTR_FLAG_LOWMED,
                        Esp32Device::static_interrupt_handler, this,
                        &interrupt_handle_);
}

void Esp32Device::set_address(uint8_t address) {
  auto dcfg = usb_->dcfg;
  dcfg &= ~USB_DEVADDR_M;
  dcfg |= ((address & USB_DEVADDR_V) << USB_DEVADDR_S);
  usb_->dcfg = dcfg;
}

bool Esp32Device::configure_ep0(uint8_t max_packet_size) {
  ESP_LOGI(LogTag, "configure EP0, MPS=%u", max_packet_size);
  // This method should only be called when EP0 is disabled and has not yet
  // been configured.  (e.g., after a bus reset)
  if (usb_->daintmsk & (USB_OUTEPMSK0_M | USB_INEPMSK0_M)) {
    ESP_LOGE(LogTag,
             "configure_ep0() called when endpoint 0 was already configured");
    return false;
  }
  assert(in_transfers_[0].status == InEPStatus::Unconfigured);
  assert(out_transfers_[0].status == OutEPStatus::Unconfigured);

  // Configure FIFO sizes
  // Configure the receive FIFO size.
  // Use 52 words (208 bytes), simply since this is the value that tinyusb
  // uses.  The comments in the tinyusb code appear to refer to data from
  // Synopsys technical reference manuals.
  usb_->grxfsiz = 52;
  // TX FIFO size
  // Control IN uses FIFO 0 with 64 bytes (16 32-bit words)
  usb_->gnptxfsiz = (16 << USB_NPTXFDEP_S) | (usb_->grxfsiz & 0x0000ffffUL);

  // Compute bits to set in the doepctl and diepctl registers
  EP0MaxPktSize mps_bits;
  if (max_packet_size == 64) {
    mps_bits = EP0MaxPktSize::Mps64;
  } else if (max_packet_size == 32) {
    mps_bits = EP0MaxPktSize::Mps32;
  } else if (max_packet_size == 16) {
    mps_bits = EP0MaxPktSize::Mps16;
  } else if (max_packet_size == 8) {
    mps_bits = EP0MaxPktSize::Mps8;
  } else {
    ESP_LOGE(LogTag, "configure_ep0() called with invalid max packet size %d",
             max_packet_size);
    return false;
  }
  constexpr int endpoint_num = 0;

  in_transfers_[endpoint_num].status = InEPStatus::Idle;
  out_transfers_[endpoint_num].status = OutEPStatus::Idle;

  // Set the doeptsiz register to enable receipt of SETUP packets
  set_bits(usb_->out_ep_reg[0].doeptsiz, USB_SUPCNT0_M);

  // Update doepctl
  // - Set the max packet size
  // - Enable NAK'ing any received OUT packets.  (It might be nice if we could
  //   tell the hardware to immediately start accepting OUT data as well, to
  //   avoid possibly NAK'ing the host's first OUT packet after a SETUP packet.
  //   However, even if we disable NAK'ing OUT packets here, the hardware
  //   automatically re-enables this flag after receipt of a SETUP packet, so
  //   there is no point in us trying to set it here.)
  // - We don't set USB_USBACTEP0; according to the comments in soc/usb_reg.h
  //   this flag appears unnecessary for EP0, as EP0 is always active.
  uint32_t doepctl = usb_->out_ep_reg[endpoint_num].doepctl;
  doepctl &= ~USB_MPS0_M;
  usb_->out_ep_reg[endpoint_num].doepctl =
      doepctl | (mps_bits << USB_MPS0_S) | USB_DO_SNAK0_M;

  // Update diepctl
  // - Set the max packet size
  // - Set the TX FIFO number to 0
  // - Clear the STALL bit
  uint32_t diepctl = usb_->in_ep_reg[endpoint_num].diepctl;
  diepctl &= ~(USB_D_MPS0_M | USB_D_TXFNUM0_M | USB_D_STALL0_M);
  usb_->in_ep_reg[endpoint_num].diepctl = diepctl | (mps_bits << USB_D_MPS0_S);

  // Enable receipt of endpoint IN and OUT interrupts
  set_bits(usb_->gintmsk, USB_IEPINTMSK_M | USB_OEPINTMSK_M);
  // Enable IN and OUT interrupts for EP0.
  usb_->daintmsk = USB_OUTEPMSK0_M | USB_INEPMSK0_M;

  return true;
}

void Esp32Device::stall_control_endpoint(uint8_t endpoint_num) {
  // Note that the EPENA / EPDIS flags are not used when stalling control
  // endpoints, we just set the STALL flag.  (The docs explicitly mention that
  // endpoint 0 cannot be disabled.  They are a little bit less clear about the
  // behavior if any other endpoint is configured as a control endpoint type.)
  set_bits(usb_->out_ep_reg[endpoint_num].doepctl, USB_STALL0_M);
  set_bits(usb_->in_ep_reg[endpoint_num].diepctl,
           USB_DI_SNAK0_M | USB_D_STALL0_M);
  flush_tx_fifo(endpoint_num);
}

XferStartResult Esp32Device::start_write(uint8_t endpoint, const void *data,
                                         uint32_t size) {
  ESP_LOGD(LogTag, "start_write() %" PRIu32 " bytes on endpoint %d", size,
           endpoint);
  const auto ep_status = in_transfers_[endpoint].status;
  if (ep_status != InEPStatus::Idle) {
    if (ep_status == InEPStatus::Busy) {
      ESP_LOGW(
          LogTag,
          "start_write called on endpoint %d while an existing transfer is "
          "still in progress",
          endpoint);
      return XferStartResult::Busy;
    }
    ESP_LOGW(LogTag, "start_write called on endpoint %d in bad state %d",
             endpoint, static_cast<int>(in_transfers_[endpoint].status));
    return XferStartResult::EndpointNotConfigured;
  }

  // Record the transfer information in in_transfers_[endpoint]
  in_transfers_[endpoint].start(data, size);

  initiate_next_write_xfer(endpoint);
  return XferStartResult::Ok;
}

XferStartResult Esp32Device::start_read(uint8_t endpoint, void *data,
                                        uint32_t size) {
  ESP_LOGD(LogTag, "start_read() up to %" PRIu32 " bytes on endpoint %d", size,
           endpoint);
  const auto ep_status = out_transfers_[endpoint].status;
  if (ep_status != OutEPStatus::Idle) {
    if (ep_status == OutEPStatus::Busy) {
      ESP_LOGW(
          LogTag,
          "start_read called on endpoint %d while an existing transfer is "
          "still in progress",
          endpoint);
      return XferStartResult::Busy;
    }
    ESP_LOGW(LogTag, "start_read called on endpoint %d in bad state %d",
             endpoint, static_cast<int>(out_transfers_[endpoint].status));
    return XferStartResult::EndpointNotConfigured;
  }

  // Record the transfer information in out_transfers_[endpoint]
  out_transfers_[endpoint].start(data, size);

  initiate_next_read_xfer(endpoint);
  return XferStartResult::Ok;
}

XferStartResult Esp32Device::ack_ctrl_in() {
  // TODO: Suppress the InXferCompleteEvent / InXferFailedEvent that occurs
  // when this OUT transfer is complete.  We mainly do this to be more similar
  // to other USB implementations like the MAX3420 which have a dedicated
  // register for ACK'ing control transfers, and which do not provide
  // notification of completion.
  return start_read(0, nullptr, 0);
}

/*
 * Overall device IN transfer steps:
 * 1) Update the DIEPTSIZ register with the number of packets and total number
 *    of bytes being sent.
 * 2) Update the DIEPCTL to enable the endpoint to start transmitting.
 * 3) Write data into the TX FIFO.  We may not be able to fit everything at
 *    once, so write as many whole packets as we can, then wait for a TXFE
 *    interrupt to signal we can write more.
 * 4) Once we have written everything to the FIFO and they have actually been
 *    consumed by the host, we will get an XFERCOMPL interrupt to signal the
 *    transfer is done.
 *
 * This function updates the DIEPTSIZ and DIEPCTL registers to start a new IN
 * transfer, and then calls write_to_fifo() to start writing the data to the
 * FIFO.  The transfer information should have already been placed in
 * in_transfers_[endpoint].
 */
void Esp32Device::initiate_next_write_xfer(uint8_t endpoint_num) {
  usb_in_endpoint_t *const in_ep = &(usb_->in_ep_reg[endpoint_num]);
  uint32_t diepctl = in_ep->diepctl;
  auto& xfer = in_transfers_[endpoint_num];

  assert(xfer.status == InEPStatus::Busy);
  // We should only invoke initiate_next_write_xfer() when the hardware
  // is not currently performing a transfer and is ready to accept a new
  // transfer.  Our own tracking of the endpoint status should ensure this.
  assert((diepctl & USB_D_EPENA1_M) == 0);

  // Compute the number of packets and bytes to send in this HW transfer
  uint16_t pkts_to_send;
  uint32_t bytes_to_send;
  const auto size_remaining = xfer.size - xfer.cur_xfer_end;
  if (size_remaining == 0) {
    // This should only ever happen for the first and only call o
    // a 0-length tranfer.
    assert(xfer.size == 0);
    pkts_to_send = 1;
    bytes_to_send = 0;
  } else {
    uint16_t max_pkt_size;
    uint16_t max_pkt_cnt;
    uint32_t max_xfer_size;
    if (endpoint_num == 0) {
      // EP0 uses a different encoding for the MPS in diepctl than other
      // endpoints.
      const auto mps_bits =
          static_cast<EP0MaxPktSize>((diepctl >> USB_D_MPS0_S) & USB_D_MPS0_V);
      max_pkt_size = get_ep0_max_packet_size(mps_bits);
      max_pkt_cnt = USB_D_PKTCNT0_V;
      max_xfer_size = USB_D_XFERSIZE1_V;
    } else {
      max_pkt_size = ((diepctl >> USB_D_MPS1_S) & Correct_USB_D_MPS1_V);
      max_pkt_cnt = Correct_USB_D_PKTCNT1_V;
      max_xfer_size = Correct_USB_D_XFERSIZE1_V;
    }

    auto max_bytes_to_send = std::min(size_remaining, max_xfer_size);
    const auto pkts_left =
        (max_bytes_to_send + (max_xfer_size - 1)) / max_xfer_size;
    if (pkts_left > max_pkt_cnt) {
      pkts_to_send = max_pkt_cnt;
      bytes_to_send = pkts_to_send * max_pkt_size;
    } else {
      pkts_to_send = pkts_left;
      bytes_to_send = max_bytes_to_send;
    }
  }

  ESP_LOGV(LogTag,
           "start IN XFER on endpoint %d: %" PRIu16 " packets, %" PRIu32
           " bytes",
           endpoint_num, pkts_to_send, bytes_to_send);
  xfer.cur_xfer_end += bytes_to_send;

  // Now start the HW transfer
  in_ep->dieptsiz =
      (pkts_to_send << USB_D_PKTCNT1_S) | (bytes_to_send << USB_D_XFERSIZE1_S);
  // TODO: for isochronous endpoints we may need to set USB_DI_SETD0PID1 or
  // USB_DI_SETD1PID1 appropriately in diepctl.
  in_ep->diepctl = diepctl | USB_D_EPENA1_M | USB_D_CNAK1_M;

  if (bytes_to_send > 0) {
    write_to_fifo(endpoint_num);
  }
}

void Esp32Device::initiate_next_read_xfer(uint8_t endpoint_num) {
  usb_out_endpoint_t *const out_ep = &(usb_->out_ep_reg[endpoint_num]);
  uint32_t doepctl = out_ep->doepctl;
  auto& xfer = out_transfers_[endpoint_num];

  assert(xfer.status == OutEPStatus::Busy);
  // We should only invoke initiate_next_write_xfer() when the hardware
  // is not currently performing a transfer and is ready to start a new
  // transfer.  Our own tracking of the endpoint status should ensure this.
  assert((doepctl & USB_EPENA1_M) == 0);

  uint16_t max_pkt_size;
  uint16_t max_pkt_cnt;
  uint32_t max_xfer_size;
  if (endpoint_num == 0) {
    // EP0 uses a different encoding for the MPS in doepctl than other
    // endpoints.
    const auto mps_bits =
        static_cast<EP0MaxPktSize>((doepctl >> USB_MPS0_S) & USB_MPS0_V);
    max_pkt_size = get_ep0_max_packet_size(mps_bits);
    max_pkt_cnt = USB_PKTCNT0_V;
    max_xfer_size = USB_XFERSIZE0_V;
  } else {
    max_pkt_size = ((doepctl >> USB_MPS1_S) & Correct_USB_MPS1_V);
    max_pkt_cnt = Correct_USB_PKTCNT1_V;
    max_xfer_size = Correct_USB_XFERSIZE1_V;
  }

  // If max_pkt_size is not a multiple of 4, it needs to be rounded up since
  // data is written into the FIFO in whole words.
  uint16_t fifo_pkt_size = (max_pkt_size + static_cast<uint16_t>(3)) & 0xfffc;

  // Compute the number of packets to ask the hardware to receive
  uint32_t pkts_to_read;
  const auto size_remaining = xfer.capacity - xfer.bytes_read;
  if (size_remaining == 0) {
    // This should only ever happen for the first and only call o
    // a 0-length tranfer.
    assert(xfer.capacity == 0);
    pkts_to_read = 1;
  } else {
    // Adjust max_pkt_cnt to ensure our full transfer cannot exceed
    // max_xfer_size if we transferred this many full packets.
    max_pkt_cnt = static_cast<uint16_t>(std::min(
        static_cast<uint32_t>(max_pkt_cnt), max_xfer_size / fifo_pkt_size));

    pkts_to_read = (size_remaining + max_pkt_size - 1) / max_pkt_size;
    if (pkts_to_read > max_pkt_cnt) {
      pkts_to_read = max_pkt_cnt;
    }
  }
  const uint32_t bytes_to_read = pkts_to_read * fifo_pkt_size;

  ESP_LOGV(LogTag,
           "start OUT XFER on endpoint %d: up to %" PRIu32 " packets, %" PRIu32
           " bytes",
           endpoint_num, pkts_to_read, bytes_to_read);

  // Now set doeptsiz and doepctl to ask the hardware to
  // start accepting OUT packets
  set_bits(out_ep->doeptsiz, (pkts_to_read << USB_PKTCNT1_S) |
                                 (bytes_to_read << USB_XFERSIZE1_S));
  set_bits(out_ep->doepctl, USB_EPENA0_M | USB_CNAK0_M);
}

uint16_t Esp32Device::get_max_in_pkt_size(uint8_t endpoint_num, uint32_t diepctl) {
  if (endpoint_num == 0) {
    const auto mps_bits =
        static_cast<EP0MaxPktSize>((diepctl >> USB_D_MPS0_S) & USB_D_MPS0_V);
    return get_ep0_max_packet_size(mps_bits);
  } else {
    return ((diepctl >> USB_D_MPS1_S) & Correct_USB_D_MPS1_V);
  }
}

uint8_t Esp32Device::get_ep0_max_packet_size(EP0MaxPktSize mps_bits) {
  switch (mps_bits) {
  case EP0MaxPktSize::Mps64:
    return 64;
  case EP0MaxPktSize::Mps32:
    return 32;
  case EP0MaxPktSize::Mps16:
    return 16;
  case EP0MaxPktSize::Mps8:
    return 8;
  }

  // Shouldn't really be possible to reach here.
  return 64;
}

/*
 * Write more data from the current in_transfers_[endpoint_num] to the FIFO.
 *
 * Will write data up to the cur_xfer_end.  Updates the cur_fifo_ptr field of
 * the transfer based on how much data was written to the FIFO.  This method
 * should only be called when cur_fifo_ptr is less than cur_xfer_end.
 *
 * If not all data could be written, this enables the TXFE interrupt.  Once the
 * TXFE interrupt is seen, write_to_fifo() should be called again to write more
 * data.
 */
void Esp32Device::write_to_fifo(uint8_t endpoint_num) {
  usb_in_endpoint_t *const in_ep = &(usb_->in_ep_reg[endpoint_num]);
  uint32_t diepctl = in_ep->diepctl;
  auto& xfer = in_transfers_[endpoint_num];

  assert((diepctl & USB_D_EPENA1_M) != 0);
  uint8_t const fifo_num = ((diepctl >> USB_D_TXFNUM1_S) & USB_D_TXFNUM1_V);
  const auto max_packet_size = get_max_in_pkt_size(endpoint_num, diepctl);
  const auto tx_words_avail =
      (in_ep->dtxfsts & USB_D_INEPTXFSPCAVAIL0_M) >> USB_D_INEPTXFSPCAVAIL0_S;
  auto tx_space_avail = tx_words_avail * 4;
  ESP_LOGV(LogTag, "write_to_fifo() on endpoint %d: tx_space_avail=%" PRIu32,
           endpoint_num, tx_space_avail);

  while (true) {
    const auto bytes_left = xfer.cur_xfer_end - xfer.cur_fifo_ptr;
    if (bytes_left <= 0) {
      break;
    }
    const uint16_t pkt_size =
        std::min(bytes_left, static_cast<uint32_t>(max_packet_size));

    if (tx_space_avail < pkt_size) {
      // We have more data to send, but no more room in the FIFO at the moment.
      // Enable the TXFE interrupt for this endpoint so we will be notified
      // when the FIFO is empty.
      ESP_LOGV(LogTag,
               "endpoint %d TX FIFO is full, enabling FIFO empty interrupt",
               endpoint_num);
      set_bits(usb_->dtknqr4_fifoemptymsk, 1 << endpoint_num);
      return;
    }

    copy_pkt_to_fifo(
        fifo_num, static_cast<const uint8_t *>(xfer.data) + xfer.cur_fifo_ptr,
        pkt_size);
    xfer.cur_fifo_ptr += pkt_size;
    tx_space_avail -= pkt_size;
    ESP_LOGV(LogTag, "copied packet of size %" PRIu16 " to endpoint %d FIFO %d",
             pkt_size, endpoint_num, fifo_num);
  }
}

/*
 * Copy a single packet into a TX FIFO.
 */
void Esp32Device::copy_pkt_to_fifo(uint8_t fifo_num, const void *data,
                                   uint16_t pkt_size) {
  volatile uint32_t *tx_fifo = usb_->fifo[fifo_num];
  const uint16_t whole_words = pkt_size / 4;
  if ((reinterpret_cast<intptr_t>(data) % 4) == 0) {
    const uint32_t* data32 = static_cast<const uint32_t*>(data);
    for (uint16_t n = 0; n < whole_words; ++n) {
      (*tx_fifo) = data32[n];
      ESP_LOGV(LogTag, "  aligned write word %" PRIu16, n);
    }
  } else {
    // Handle writing unaligned input data
    const uint8_t* data8 = static_cast<const uint8_t*>(data);
    const uint16_t end = whole_words * 4;
    for (uint16_t n = 0; n < end; n += 4) {
      uint32_t word = data8[n] | (data8[n + 1] << 8) | (data8[n + 2] << 16) |
                      (data8[n + 3] << 24);
      (*tx_fifo) = word;
      ESP_LOGV(LogTag, "  unaligned write word %" PRIu16, n);
    }
  }

  // Handle any remaining data less than a full word
  const uint16_t idx = whole_words * 4;
  if (idx < pkt_size) {
    ESP_LOGV(LogTag, "  write tail difference %" PRIu16 " - %" PRIu16, pkt_size,
             idx);
    const uint8_t* tail = static_cast<const uint8_t*>(data) + idx;
    uint32_t word = tail[0];
    if (idx + 1 < pkt_size) {
      word |= (tail[1] << 8);
    }
    if (idx + 2 < pkt_size) {
      word |= (tail[2] << 16);
    }
    (*tx_fifo) = word;
  }
}

void Esp32Device::stall_in_endpoint(uint8_t endpoint_num) {
  assert(endpoint_num != 0);
  usb_in_endpoint_t *const in_ep = &(usb_->in_ep_reg[endpoint_num]);

  if ((endpoint_num == 0) || !(in_ep->diepctl & USB_D_EPENA1_M)) {
    // If the endpoint is not currently enabled, we just have to set the STALL
    // flag, and don't need to disable it.
    set_bits(in_ep->diepctl, USB_DI_SNAK1_M | USB_D_STALL1_M);
  } else {
    // Set USB_DI_SNAK1_M to NAK transfers.
    set_bits(in_ep->diepctl, USB_DI_SNAK1_M);
    while ((in_ep->diepint & USB_DI_SNAK1_M) == 0) {
      // Busy loop until we observe the USB_DI_SNAK1_M interrupt flag.
      // TODO: It would ideally be nicer to make this API asynchronous, and
      // finish the operation in the interrupt handler rather than busy
      // looping.
    }

    // Disable the endpoint and also set the STALL flag.
    // NAK is also still set.
    set_bits(in_ep->diepctl,
             USB_DI_SNAK1_M | USB_D_STALL1_M | USB_D_EPDIS1_M);
    while ((in_ep->diepint & USB_D_EPDISBLD0_M) == 0) {
      // Busy loop until we observe the USB_D_EPDISBLD0_M interrupt flag.
    }
    // Clear the USB_D_EPDISBLD0_M interrupt.
    in_ep->diepint = USB_D_EPDISBLD0_M;
  }

  // Flush the transmit FIFO.
  flush_tx_fifo(endpoint_num);
}

void Esp32Device::flush_tx_fifo(uint8_t endpoint_num) {
  usb_in_endpoint_t *const in_ep = &(usb_->in_ep_reg[endpoint_num]);
  uint8_t const fifo_num =
      ((in_ep->diepctl >> USB_D_TXFNUM1_S) & USB_D_TXFNUM1_V);
  set_bits(usb_->grstctl, (fifo_num << USB_TXFNUM_S) | USB_TXFFLSH_M);
  while ((usb_->grstctl & USB_TXFFLSH_M) != 0) {
    // Busy loop until the FIFO has been cleared.
  }
}

void Esp32Device::stall_out_endpoint(uint8_t endpoint_num) {
  assert(endpoint_num != 0);
  usb_out_endpoint_t *const out_ep = &(usb_->out_ep_reg[endpoint_num]);

  if (!(out_ep->doepctl & USB_EPENA0_M)) {
    // If the endpoint is not currently enabled, nothing else to do besides set
    // the STALL flag.
    set_bits(out_ep->doepctl, USB_STALL0_M);
  } else {
    // The Global NAK flag apparently must be enabled to STALL an OUT
    // endpoint.  Enable this flag, and wait for it to take effect.
    set_bits(usb_->dctl, USB_SGOUTNAK_M);
    while ((usb_->gintsts & USB_GOUTNAKEFF_M) == 0) {
      // Busy loop until we have seen the USB_GOUTNAKEFF_M interrupt.
      // TODO: It would ideally be nicer to make this API asynchronous, and
      // finish the operation in the interrupt handler rather than busy
      // looping.
    }

    // Disable the endpoint, and set the STALL flag.
    set_bits(out_ep->doepctl, USB_STALL0_M | USB_EPDIS0_M);
    while ((out_ep->doepint & USB_EPDISBLD0_M) == 0) {
      // Busy loop until we have seen the endpoint disabled interrupt.
    }
    out_ep->doepint = USB_EPDISBLD0_M;

    // Clear the Global NAK flag to allow other OUT endpoints to continue
    // functioning.
    set_bits(usb_->dctl, USB_CGOUTNAK_M);
  }
}

void Esp32Device::static_interrupt_handler(void *arg) {
  auto *dev = static_cast<Esp32Device *>(arg);
  dev->intr_main();
}

void Esp32Device::intr_main() {
  const uint32_t mask = usb_->gintmsk;
  const uint32_t int_status = (usb_->gintsts & mask);
  ISR_LOGD("USB interrupt: 0x%02" PRIx32 " 0x%02" PRIx32, mask, int_status);

  // Check int_status for all of the interrupt bits that we need to handle.
  // As we process each interrupt, set that bit in gintsts to clear the
  // interrupt.  Do this before sending the event back to the main task, as
  // send_event_from_isr() may yield back to another higher priority task
  // before returning.

  if (int_status & (USB_USBRST_M | USB_RESETDET_M)) {
    // USB_USBRST_M indicates a reset.
    // USB_RESETDET_M indicates a reset detected while in suspend mode.
    //
    // These two tend to be asserted together when first initializing the bus
    // as a device.
    usb_->gintsts = USB_USBRST_M | USB_RESETDET_M;
    intr_bus_reset();
  }

  if (int_status & USB_ENUMDONE_M) {
    usb_->gintsts = USB_ENUMDONE_M;
    intr_enum_done();
  }

  if (int_status & USB_ERLYSUSP_M) {
    ISR_LOGV("USB int: early suspend");
    // TODO: might need to check for USB_ERRTICERR in the dsts register?
    // According to some of the STM docs it looks like erratic errors can
    // trigger an early suspend interrupt with the erratic error status flag
    // set.  We need to then do a soft disconnect to recover.
    usb_->gintsts = USB_ERLYSUSP_M;
  }

  if (int_status & USB_USBSUSP_M) {
    ISR_LOGV("USB int: suspend");
    usb_->gintsts = USB_USBSUSP_M;
    send_event_from_isr<SuspendEvent>();
  }

  if (int_status & USB_WKUPINT_M) {
    ISR_LOGV("USB int: resume");
    usb_->gintsts = USB_WKUPINT_M;
    send_event_from_isr<ResumeEvent>();
  }

  if (int_status & USB_DISCONNINT_M) {
    usb_->gintsts = USB_DISCONNINT_M;
    ISR_LOGV("USB disconnect");
  }

  if (int_status & USB_SOF_M) {
    // Start of frame.
    //
    // We only enable SOF interupts to detect when the bus has resumed after we
    // have triggered a remote wakeup.  Re-disable SOF interrupts, and then
    // send a resume event to the main event handler.
    ISR_LOGV("USB int: start of frame");
    usb_->gintsts = USB_SOF_M;
    clear_bits(usb_->gintmsk, USB_SOFMSK_M);
    send_event_from_isr<ResumeEvent>();
  }

  if (int_status & USB_RXFLVI_M) {
    ISR_LOGV("USB int: RX FIFO level");
    // No need to update usb_->gintsts to indicate we have handled the
    // interrupt; this will be cleared when we read from the fifo.

    // Disable the RXFLVL interrupt.  The main USB task will re-enable it once
    // it has processed data from the FIFO.
    clear_bits(usb_->gintmsk, USB_RXFLVIMSK_M);
    send_event_from_isr<RxFifoNonEmpty>();
  }

  if (int_status & USB_OEPINT_M) {
    // OUT endpoint interrupt
    ISR_LOGV("USB int: OUT endpoint");
    // No need to update usb_->gintsts to indicate we have handled the
    // interrupt; intr_out_endpoint() will update the endpoint doepint
    // register instead.
    intr_out_endpoint_main();
  }

  if (int_status & USB_IEPINT_M) {
    // IN endpoint interrupt
    ISR_LOGV("USB int: IN endpoint");
    // No need to update usb_->gintsts to indicate we have handled the
    // interrupt; intr_in_endpoint() will update the endpoint diepint
    // register instead.
    intr_in_endpoint_main();
  }

  if (int_status & USB_MODEMIS_M) {
    // This interrupt means we tried to access a host-mode-only register
    // while in device mode.  This should only happen if we have a bug.
    ISR_LOGD("bug: USB_MODEMIS_M interrupt fired");
    usb_->gintsts = USB_MODEMIS_M;
  }

  if (int_status & USB_OTGINT_M) {
    usb_->gintsts = USB_OTGINT_M;
    ISR_LOGD("USB OTG interrupt");
  }

  // Clear other interrupt bits that we do not handle.
  // (This is probably unnecessary since never enable these in gintmsk)
  usb_->gintsts = USB_CURMOD_INT_M | USB_NPTXFEMP_M | USB_GINNAKEFF_M |
                  USB_GOUTNAKEFF | USB_ISOOUTDROP_M | USB_EOPF_M | USB_EPMIS_M |
                  USB_INCOMPISOIN_M | USB_INCOMPIP_M | USB_FETSUSP_M |
                  USB_PRTLNT_M | USB_HCHLNT_M | USB_PTXFEMP_M |
                  USB_CONIDSTSCHNG_M | USB_SESSREQINT_M;
  ISR_LOGV("USB interrupt 0x%02" PRIx32 " done", int_status);
}

void Esp32Device::intr_bus_reset() {
  ISR_LOGV("USB int: reset");
  // Configure all endpoints to NAK any new packets
  all_endpoints_nak();
  // Clear the device address
  clear_bits(usb_->dcfg, USB_DEVADDR_M);

  // Disable all endpoint interrupts
  usb_->daintmsk = 0;
  usb_->doepmsk = USB_SETUPMSK_M | USB_XFERCOMPLMSK;
  usb_->diepmsk = USB_TIMEOUTMSK_M | USB_DI_XFERCOMPLMSK_M;

  send_event_from_isr<BusResetEvent>();
}

void Esp32Device::intr_enum_done() {
  // The Device Status register (DSTS_REG) contains the enumerated speed
  // We pretty much always expect this to be Speed::Full48Mhz.  In theory if we
  // are connected to a host that only supports low speed we could see
  // Speed::Low6Mhz.
  uint32_t enum_spd = (usb_->dsts >> USB_ENUMSPD_S) & (USB_ENUMSPD_V);
  ISR_LOGV("USB int: enumeration done; speed=%d", enum_spd);

  UsbSpeed speed;
  if (enum_spd == Speed::Full48Mhz) {
    speed = UsbSpeed::Full;
  } else {
    speed = UsbSpeed::Low;
  }

  send_event_from_isr<BusEnumDone>(speed);
}

DeviceEvent Esp32Device::process_rx_fifo() {
  // Process entries from the FIFO until we encounter an event to return to the
  // application, or until the FIFO is empty.
  while (true) {
    // Pop the control word from the top of the RX FIFO
    uint32_t const ctrl_word = usb_->grxstsp;
    auto event = process_one_rx_entry(ctrl_word);
    if (!std::holds_alternative<NoEvent>(event)) {
      // If the FIFO still has data to process, add another RxFifoNonEmpty
      // event to the front of the queue so we will check if there is still
      // more to process in the RX FIFO the next time wait_for_event() is
      // called.
      if ((usb_->gintsts & USB_RXFLVI_M) == 0) {
        pending_event_ = RxFifoNonEmpty();
      } else {
        // Re-enable the RXFLVL interrupt to be notified when there is more
        // data in the fifo.
        set_bits(usb_->gintmsk, USB_RXFLVIMSK_M);
      }
      return event;
    }

    // If USB_RXFLVI_M is no longer set in GINTSTS, there is nothing more to
    // read from the RX FIFO.
    if ((usb_->gintsts & USB_RXFLVI_M) == 0) {
      // Re-enable the RXFLVL interrupt to be notified when there is more
      // data in the fifo.
      set_bits(usb_->gintmsk, USB_RXFLVIMSK_M);
      return NoEvent(NoEventReason::HwProcessing);
    }
  }
}

/*
 * Process one entry from the RX FIFO
 */
DeviceEvent Esp32Device::process_one_rx_entry(uint32_t ctrl_word) {
  // USB_PKTSTS values from the comments in soc/usb_reg.h
  enum Pktsts : uint32_t {
    GlobalOutNak = 1,
    PktReceived = 2,
    TxComplete = 3,
    SetupComplete = 4,
    DataToggleError = 5, // host mode only
    SetupReceived = 6,
    ChannelHalted = 7, // host mode only
  };

  uint8_t const pktsts = (ctrl_word & USB_PKTSTS_M) >> USB_PKTSTS_S;
  switch (pktsts) {
  case Pktsts::PktReceived: {
    uint8_t const endpoint_num = (ctrl_word & USB_CHNUM_M) >> USB_CHNUM_S;
    uint16_t const byte_count = (ctrl_word & USB_BCNT_M) >> USB_BCNT_S;
    ESP_LOGD(LogTag, "USB RX: OUT packet received on EP%u; size=%u",
             endpoint_num, byte_count);
    receive_packet(endpoint_num, byte_count);
    return NoEvent(NoEventReason::HwProcessing);
  }
  case Pktsts::SetupComplete: {
    // SetupComplete is generated by the HW after the SETUP phase is done.
    // From reading the documentation, it sounds like the HW may wait to see
    // and NAK the first IN or OUT token from the host before generating this
    // interrupt, but it isn't 100% clear about this.  It seems like the HW was
    // designed this way in order to make it easier for the software to detect
    // and discard duplicate SETUP packets that were retransmitted due to the
    // host thinking there may have been a transmission error.
    //
    // Many other USB HW implementations don't behave this way, so our higher
    // level application code still needs to explicitly handle retransmitted
    // SETUP packets on other hardware.
    uint8_t const endpoint_num = (ctrl_word & USB_CHNUM_M) >> USB_CHNUM_S;
    ESP_LOGD(LogTag, "USB RX: setup done EP%d", endpoint_num);

    // Reset the doeptsiz register to indicate that we can receive more SETUP
    // packets.
    set_bits(usb_->out_ep_reg[endpoint_num].doeptsiz, USB_SUPCNT0_M);
    return NoEvent(NoEventReason::HwProcessing);
  }
  case Pktsts::SetupReceived: {
    // We store the setup packet in a dedicated member variable rather than a
    // normal RX buffer.  The host may retransmit SETUP packets, and the packet
    // data itself does not indicate if this is a retransmission.  If we
    // receive multiple SETUP packets in a row before notifying the main task
    // of receipt, we simply overwrite the data from previous packets.
    //
    // We should receive a SetupComplete interrupt after SetupReceived, follwed
    // by an OUT interrupt with the USB_SETUP0_M flag set.
    //
    // We wait to inform the main USB task of the SETUP packet until we get the
    // final OUT interrupt.  We don't want to inform the software yet, since
    // the HW will reset doeptsiz and doepctl when the setup OUT interrupt is
    // generated, and we don't want the software to attempt to start its own
    // new out transfer before we have finished processing the out interrupt
    // for this setup event.
    volatile uint32_t *rx_fifo = usb_->fifo[0];

    // assert that the byte count is exactly the length of a SETUP packet.
    assert(((ctrl_word & USB_BCNT_M) >> USB_BCNT_S) == 8);

    setup_packet_.u32[0] = (*rx_fifo);
    setup_packet_.u32[1] = (*rx_fifo);
    ESP_LOGD(LogTag, "USB RX: setup packet: 0x%08" PRIx32" 0x%08" PRIx32,
             setup_packet_.u32[0], setup_packet_.u32[1]);
    return NoEvent(NoEventReason::HwProcessing);
  }
  case Pktsts::GlobalOutNak:
    ESP_LOGD(LogTag, "USB RX: Global OUT NAK effective");
    break;
  case Pktsts::TxComplete:
    // This signals that all data for a single OUT transfer was read.
    // This will occur when either
    // - We read exactly as many packets and bytes specified in doeptsiz
    // - The host sent a short packet, indicating end of transfer
    //
    // Now that we have popped this entry off the RX FIFO, an out endpoint
    // interrupt will be generated with the USB_XFERCOMPL flag set.
    // We don't do any other processing here, and we wait until that interrupt
    // to handle the transfer completion.
    ESP_LOGD(LogTag, "USB RX: OUT transfer done");
    break;
  default:
    ESP_LOGE(LogTag, "USB RX: unexpected pktsts value: %x", pktsts);
    break;
  }

  return NoEvent(NoEventReason::HwProcessing);
}

void Esp32Device::intr_out_endpoint_main() {
  for (uint8_t epnum = 0; epnum < USB_OUT_EP_NUM; ++epnum) {
    if (usb_->daint & (1 << (16 + epnum))) {
      intr_out_endpoint(epnum);
    }
  }
}

void Esp32Device::intr_in_endpoint_main() {
  for (uint8_t epnum = 0; epnum < USB_IN_EP_NUM; ++epnum) {
    if (usb_->daint & (1 << epnum)) {
      intr_in_endpoint(epnum);
    }
  }
}

void Esp32Device::intr_out_endpoint(uint8_t endpoint_num) {
  const auto doepint = usb_->out_ep_reg[endpoint_num].doepint;
  ISR_LOGD("USB OUT interrupt: EP%u DOEPINT=%" PRIx32, endpoint_num, doepint);

  // Clear all of the interrupt flags we handle
  set_bits(usb_->out_ep_reg[endpoint_num].doepint,
           doepint & (USB_SETUP0_M | USB_STUPPKTRCVD0_M | USB_XFERCOMPL0_M));
  send_event_from_isr<OutEndpointInterrupt>(endpoint_num, doepint);
}

void Esp32Device::intr_in_endpoint(uint8_t endpoint_num) {
  const auto diepint = usb_->in_ep_reg[endpoint_num].diepint;
  ISR_LOGD("USB IN interrupt: EP%u DIEPINT=%" PRIx32, endpoint_num, diepint);

  // If the TX FIFO is empty, clear the interrupt mask to stop receiving this
  // interrupt until we re-enable it.
  if (diepint & USB_D_TXFEMP0_M) {
    clear_bits(usb_->dtknqr4_fifoemptymsk, 1 << endpoint_num);
  }
  // Clear all of the interrupt flags we handle
  set_bits(usb_->in_ep_reg[endpoint_num].diepint,
           diepint & (USB_D_XFERCOMPL0_M | USB_D_TXFEMP0_M | USB_D_TIMEOUT0_M));
  send_event_from_isr<InEndpointInterrupt>(endpoint_num, diepint);
}

void Esp32Device::receive_packet(uint8_t endpoint_num, uint16_t packet_size) {
  // We currently operate in what the ESP32-S3 Technical Reference Manual
  // describes as "Slave mode", and manually read data from the RX FIFO.
  //
  // The device also supports using DMA to directly place data into our memory.
  // It would probably be nicer to use DMA mode in the future, but the
  // documentation for DMA functionality is somewhat limited and I haven't
  // spent much time playing around with it.

  // All RX transfers are performed using FIFO 0.
  volatile uint32_t *rx_fifo = usb_->fifo[0];

  auto &xfer = out_transfers_[endpoint_num];
  if (xfer.status != OutEPStatus::Busy) {
    // It's possible that this could happen if we recently processed
    // application-initiated request to abort the transfer and this raced
    // with us receiving the interrupt.
    ESP_LOGD(LogTag,
             "OUT data available endpoint %d (%" PRIu16 " bytes), but transfer "
             "is no longer in progress",
             endpoint_num, packet_size);
    // Read and discard the data
    for (size_t n = 0; n < packet_size; n += sizeof(uint32_t)) {
      const uint32_t ignored = (*rx_fifo);
      static_cast<void>(ignored);
    }
    return;
  }

  const auto words_in_fifo =
      (packet_size + sizeof(uint32_t) - 1) / sizeof(uint32_t);
  auto capacity_left = xfer.capacity - xfer.bytes_read;

  // Read as many full words as are available and fit in the destination buffer
  const uint32_t full_words_to_copy =
      std::min(static_cast<uint32_t>(packet_size), capacity_left) /
      sizeof(uint32_t);
  ESP_LOGV(LogTag, "EP%d out: copy %" PRIu32 " full words", endpoint_num,
           full_words_to_copy);
  auto* buf = static_cast<uint8_t*>(xfer.data) + xfer.bytes_read;
  xfer.bytes_read += packet_size;
  for (uint32_t n = 0; n < full_words_to_copy; ++n) {
    const uint32_t word = (*rx_fifo);
    // Use memcpy() since buf may not be word-aligned.
    // Compilers should be smart about optimizing small fixed-size memcpy()
    // calls into direct memory accesses when possible calls these days.
    memcpy(buf, &word, sizeof(uint32_t));
    buf += sizeof(uint32_t);
  }

  // Return now if we have read everything from the FIFO
  if (full_words_to_copy == words_in_fifo) {
    assert(packet_size % sizeof(uint32_t) == 0);
    return;
  }
  const auto bytes_left = packet_size - (full_words_to_copy * sizeof(uint32_t));
  assert(bytes_left > 0);
  capacity_left -= sizeof(uint32_t) * full_words_to_copy;

  // We may have more capacity for a partial packet
  auto words_read = full_words_to_copy;
  if (capacity_left > 0) {
    ESP_LOGV(LogTag, "EP%d out: copy partial word of %" PRIu32 " bytes",
             endpoint_num, std::min(capacity_left, bytes_left));
    assert(capacity_left < sizeof(uint32_t));
    const uint32_t word = (*rx_fifo);
    memcpy(buf, &word, std::min(capacity_left, bytes_left));
    ++words_read;
  }

  // Read and discard any remaining words in the FIFO if we have a buffer
  // overrun
  while (words_read < words_in_fifo) {
    assert(xfer.bytes_read > xfer.capacity);
    const uint32_t word = (*rx_fifo);
    static_cast<void>(word);
  }
}

} // namespace ausb
