// Copyright (c) 2023, Adam Simpkins
#include "ausb/esp/Esp32Device.h"

#include <esp_check.h>
#include <esp_log.h>
#include <hal/usb_hal.h>
#include <soc/usb_otg_periph.h>
#include <soc/usb_periph.h>
#include <soc/usb_pins.h>

#include <atomic>

#define AUSB_VERBOSE_LOGGING

#ifdef AUSB_VERBOSE_LOGGING
#define DBG_ISR_LOG(arg, ...) ESP_EARLY_LOGI(LogTag, arg, ##__VA_ARGS__)
#else
#define DBG_ISR_LOG(arg, ...) (static_cast<void>(0))
#endif

namespace {
const char *LogTag = "ausb.esp32";

inline void set_bits(volatile uint32_t &value, uint32_t bits) {
  value = (value | bits);
}
inline void clear_bits(volatile uint32_t &value, uint32_t bits) {
  value = (value & ~bits);
}
} // unnamed namespace

namespace ausb {

Esp32Device::~Esp32Device() {
  reset();
}

DeviceEvent Esp32Device::wait_for_event(std::chrono::milliseconds timeout) {
  TickType_t max_wait = pdMS_TO_TICKS(timeout.count());
  DeviceEvent event;
  if (xQueueReceive(event_queue_, &event, max_wait) == pdTRUE) {
    return event;
  } else {
    return NoEvent();
  }
}

void Esp32Device::send_event_from_isr(const DeviceEvent& event) {
  BaseType_t higher_prio_task_woken;
  BaseType_t res =
      xQueueSendToBackFromISR(event_queue_, &event, &higher_prio_task_woken);
  portYIELD_FROM_ISR(higher_prio_task_woken);

  if (res != pdPASS) {
    ESP_EARLY_LOGE(LogTag, "USB event queue full; unable to send event");
    // TODO: we probably should set an atomic flag here that wait_for_event()
    // can then check.  We probably just need to reset the USB device after
    // this occurs.
  }
}

esp_err_t Esp32Device::init(EspPhyType phy_type,
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
  // - enable interrupt when TxFIFO is empty
  // - global interrupts enable flag
  set_bits(usb_->gahbcfg, USB_NPTXFEMPLVL_M | USB_GLBLLNTRMSK_M);
  // USB configuration:
  // - Force device mode (rather than OTG)
  set_bits(usb_->gusbcfg, USB_FORCEDEVMODE_M);
  // OTG configuration:
  // - disable overrides
  clear_bits(usb_->gotgctl,
             USB_BVALIDOVVAL_M | USB_BVALIDOVEN_M | USB_VBVALIDOVVAL_M);

  // Configure all endpoints to NAK
  all_endpoints_nak();

  // Interrupt configuration
  usb_->gintmsk = 0;          // mask all interrupts
  usb_->gotgint = 0xffffffff; // clear OTG interrupts
                              // (even though we leave OTG interrupts disabled)
  usb_->gintsts = 0xffffffff; // clear pending interrupts
  usb_->gintmsk = USB_MODEMISMSK_M | USB_RXFLVIMSK_M | USB_ERLYSUSPMSK_M |
                 USB_USBSUSPMSK_M | USB_USBRSTMSK_M | USB_ENUMDONEMSK_M |
                 USB_RESETDETMSK_M | USB_DISCONNINTMSK_M;

  ESP_RETURN_ON_ERROR(
      enable_interrupts(), LogTag, "error enabling USB interrupts");

  // Enable the data line pull-up to connect to the bus.
  clear_bits(usb_->dctl, USB_SFTDISCON_M);

  return ESP_OK;
}

void Esp32Device::reset() {
  usb_->gintmsk = 0; // Mask all interrupts
  set_bits(usb_->dctl, USB_SFTDISCON_M); // Disable the data pull-up line

  if (interrupt_handle_) {
    esp_intr_free(interrupt_handle_);
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

void Esp32Device::static_interrupt_handler(void *arg) {
  auto *dev = static_cast<Esp32Device *>(arg);
  dev->interrupt_handler();
}

void Esp32Device::interrupt_handler() {
  const uint32_t mask = usb_->gintmsk;
  const uint32_t int_status = (usb_->gintsts & mask);
  // TODO: intr_count is only used for initial debugging; remove it later
  static std::atomic<unsigned int> s_intr_count;
  auto intr_count = s_intr_count.fetch_add(1);
  DBG_ISR_LOG("USB interrupt %u: 0x%02" PRIx32 " 0x%02" PRIx32,
              intr_count, mask, int_status);

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
    bus_reset();
  }

  if (int_status & USB_ENUMDONE_M) {
    usb_->gintsts = USB_ENUMDONE_M;
    enum_done();
  }

  if (int_status & USB_ERLYSUSPMSK_M) {
    DBG_ISR_LOG("USB int: early suspend");
    usb_->gintsts = USB_ERLYSUSPMSK_M;
  }

  if (int_status & USB_USBSUSP_M) {
    DBG_ISR_LOG("USB int: suspend");
    usb_->gintsts = USB_USBSUSP_M;
    send_event_from_isr<SuspendEvent>();
  }

  if (int_status & USB_WKUPINT_M) {
    DBG_ISR_LOG("USB int: resume");
    usb_->gintsts = USB_WKUPINT_M;
    send_event_from_isr<ResumeEvent>();
  }

  if (int_status & USB_SOF_M) {
    // Start of frame.
    //
    // We only enable SOF interupts to detect when the bus has resumed after we
    // have triggered a remote wakeup.  Re-disable SOF interrupts, and then
    // send a resume event to the main event handler.
    DBG_ISR_LOG("USB int: start of frame");
    usb_->gintsts = USB_SOF_M;
    clear_bits(usb_->gintmsk, USB_SOFMSK_M);
    send_event_from_isr<ResumeEvent>();
  }

  if (int_status & USB_RXFLVI_M) {
    DBG_ISR_LOG("USB int: RX FIFO level");
    // No need to update usb_->gintsts to indicate we have handled the
    // interrupt; this will be cleared when we read from the fifo.

    // Disable the RXFLVL interrupt while reading data from the FIFO
    clear_bits(usb_->gintmsk, USB_RXFLVIMSK_M);
    rx_fifo_nonempty();
    set_bits(usb_->gintmsk, USB_RXFLVIMSK_M);
  }

  if (int_status & USB_OEPINT_M) {
    // OUT endpoint interrupt
    DBG_ISR_LOG("USB int: OUT endpoint");
    // No need to update usb_->gintsts to indicate we have handled the
    // interrupt; handle_out_ep_interrupt() will update the endpoint doepint
    // register instead.
    handle_out_ep_interrupt();
  }

  if (int_status & USB_IEPINT_M) {
    // IN endpoint interrupt
    DBG_ISR_LOG("USB int: IN endpoint");
    // No need to update usb_->gintsts to indicate we have handled the
    // interrupt; handle_in_ep_interrupt() will update the endpoint diepint
    // register instead.
    handle_in_ep_interrupt();
  }

  // Clear other interrupt bits that we do not handle.
  usb_->gintsts = USB_CURMOD_INT_M | USB_MODEMIS_M | USB_NPTXFEMP_M |
                  USB_GINNAKEFF_M | USB_GOUTNAKEFF | USB_ERLYSUSP_M |
                  USB_ISOOUTDROP_M | USB_EOPF_M | USB_EPMIS_M |
                  USB_INCOMPISOIN_M | USB_INCOMPIP_M | USB_FETSUSP_M |
                  USB_PTXFEMP_M;
  DBG_ISR_LOG("USB interrupt %u done", intr_count);
}

void Esp32Device::bus_reset() {
  DBG_ISR_LOG("USB int: reset");
  all_endpoints_nak();

  // clear the device address
  clear_bits(usb_->dcfg, USB_DEVADDR_M);

  usb_->daintmsk = USB_OUTEPMSK0_M | USB_INEPMSK0_M;
  usb_->doepmsk = USB_SETUPMSK_M | USB_XFERCOMPLMSK;
  usb_->diepmsk = USB_TIMEOUTMSK_M | USB_DI_XFERCOMPLMSK_M;

  // The Espressif tinyusb code uses 52 for the shared RX fifo size.
  // Note that this is 52 4-byte words, or 208 bytes total.
  //
  // They reference the "USB Data FIFOs" section of the reference manual,
  // also the technical reference manual copy I have (v1.0, which was just
  // released a few weeks ago) does not appear to explicitly document the RX
  // FIFO size recommendations that they reference.
  set_bits(usb_->grstctl, 0x10 << USB_TXFNUM_S); // fifo 0x10,
  set_bits(usb_->grstctl, USB_TXFFLSH_M);        // Flush fifo
  usb_->grxfsiz = 52;

  // Control IN uses FIFO 0 with 64 bytes ( 16 32-bit word )
  usb_->gnptxfsiz = (16 << USB_NPTXFDEP_S) | (usb_->grxfsiz & 0x0000ffffUL);

  // Ready to receive SETUP packet.
  set_bits(usb_->out_ep_reg[0].doeptsiz, USB_SUPCNT0_M);

  set_bits(usb_->gintmsk, USB_IEPINTMSK_M | USB_OEPINTMSK_M);

  send_event_from_isr<BusResetEvent>();
}

void Esp32Device::enum_done() {
  // The Device Status register (DSTS_REG) contains the enumerated speed
  // We pretty much always expect this to be Speed::Full48Mhz.  In theory if we
  // are connected to a host that only supports low speed we could see
  // Speed::Low6Mhz.
  uint32_t enum_spd = (usb_->dsts >> USB_ENUMSPD_S) & (USB_ENUMSPD_V);
  DBG_ISR_LOG("USB int: enumeration done; speed=%d", enum_spd);

#if 0
  uint16_t max_ep0_packet_size;
#endif
  UsbSpeed speed;
  if (enum_spd == Speed::Full48Mhz) {
    // Max packet size is 64 bytes
    clear_bits(usb_->in_ep_reg[0].diepctl, USB_D_MPS0_V);
#if 0
    max_ep0_packet_size = 64;
#endif
    speed = UsbSpeed::Full;
  } else {
    // Max packet size is 8 bytes
    set_bits(usb_->in_ep_reg[0].diepctl, USB_D_MPS0_V);
#if 0
    max_ep0_packet_size = 8;
#endif
    speed = UsbSpeed::Low;
  }
  // Clear the stall bit
  clear_bits(usb_->in_ep_reg[0].diepctl, USB_D_STALL0_M);

#if 0
  xfer_status_[0].out.max_size = max_ep0_packet_size;
  xfer_status_[0].in.max_size = max_ep0_packet_size;
#endif
  send_event_from_isr<BusEnumDone>(speed);
}

void Esp32Device::rx_fifo_nonempty() {
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

  uint32_t const ctl_word = usb_->grxstsp;
  uint8_t const pktsts = (ctl_word & USB_PKTSTS_M) >> USB_PKTSTS_S;
  switch (pktsts) {
  case Pktsts::PktReceived: {
    uint8_t const endpoint_num = (ctl_word & USB_CHNUM_M) >> USB_CHNUM_S;
    uint16_t const byte_count = (ctl_word & USB_BCNT_M) >> USB_BCNT_S;
    DBG_ISR_LOG("USB RX: OUT packet received; size=%u", byte_count);
    receive_packet(endpoint_num, byte_count);
    break;
  }
  case Pktsts::SetupComplete: {
    // Setup OUT packet received.  After this we should receive an OUT endpoint
    // interrupt with the SETUP bit set.  We generate an event to handle the
    // SetupPacket there.
    uint8_t const endpoint_num = (ctl_word & USB_CHNUM_M) >> USB_CHNUM_S;
    DBG_ISR_LOG("USB RX: setup done");
    // Set the USB_SUPCNT0_M bits in the doeptsiz register, to indicate that
    // we can receive up to 3 DATA packets in this setup transfer.
    set_bits(usb_->out_ep_reg[endpoint_num].doeptsiz, USB_SUPCNT0_M);
    break;
  }
  case Pktsts::SetupReceived: {
    // We should receive a SetupComplete interrupt after SetupReceived.
    // We simply store the setup packet data here.
    volatile uint32_t *rx_fifo = usb_->fifo[0];
    setup_packet_.u32[0] = (*rx_fifo);
    setup_packet_.u32[1] = (*rx_fifo);
    DBG_ISR_LOG("USB RX: setup packet: 0x%08x 0x%08x", setup_packet_.u32[0],
                setup_packet_.u32[1]);
    break;
  }
  case Pktsts::GlobalOutNak:
    DBG_ISR_LOG("USB RX: Global OUT NAK");
    break;
  case Pktsts::TxComplete:
    DBG_ISR_LOG("USB RX: OUT packet done");
    break;
  default:
    ESP_EARLY_LOGE(LogTag, "USB RX: unexpected pktsts value: %x", pktsts);
    break;
  }
}

void Esp32Device::handle_out_ep_interrupt() {
  for (uint8_t epnum = 0; epnum < USB_OUT_EP_NUM; ++epnum) {
    if (usb_->daint & (1 << (16 + epnum))) {
      out_endpoint_interrupt(epnum);
    }
  }
}

void Esp32Device::handle_in_ep_interrupt() {
  for (uint8_t epnum = 0; epnum < USB_IN_EP_NUM; ++epnum) {
    if (usb_->daint & (1 << epnum)) {
      in_endpoint_interrupt(epnum);
    }
  }
}

void Esp32Device::out_endpoint_interrupt(uint8_t epnum) {
  // Setup packet receipt done
  // This should presumably only happen for endpoint 0.
  if ((usb_->out_ep_reg[epnum].doepint & USB_SETUP0_M)) {
    DBG_ISR_LOG("USB: SETUP receive complete on EP%u", epnum);
    // Clear the interrupt bits
    usb_->out_ep_reg[epnum].doepint = USB_STUPPKTRCVD0_M | USB_SETUP0_M;
    send_event_from_isr<SetupPacket>(setup_packet_.setup);
  }

  // OUT transaction complete (one packet received)
  if (usb_->out_ep_reg[epnum].doepint & USB_XFERCOMPL0_M) {
    DBG_ISR_LOG("USB: OUT transaction complete on EP%u", epnum);
    // Clear the interrupt bit
    usb_->out_ep_reg[epnum].doepint = USB_XFERCOMPL0_M;

#if 0
    auto &xfer = xfer_status_[epnum].out;
    // Transfer complete if short packet or total length is received
    if (xfer.recv_complete) {
      send_event_from_isr<OutXferCompleteEvent>(epnum, xfer.received_bytes);
    } else {
      // Enable receiving another packet on this endpoint
      set_bits(usb_->out_ep_reg[epnum].doeptsiz,
               USB_PKTCNT0_M |
                   ((xfer.max_size & USB_XFERSIZE0_V) << USB_XFERSIZE0_S));
      set_bits(usb_->out_ep_reg[epnum].doepctl, USB_EPENA0_M | USB_CNAK0_M);
    }
#endif
  }
}

void Esp32Device::in_endpoint_interrupt(uint8_t epnum) {
  // TODO
  ESP_EARLY_LOGE(LogTag, "TODO: process IN endpoint interrupt");
}

void Esp32Device::receive_packet(uint8_t endpoint_num, uint16_t packet_size) {
  // We currently operate in what the ESP32-S3 Technical Reference Manual
  // describes as "Slave mode", and manually read data from the RX FIFO.
  //
  // The device also supports using DMA to directly place data into our memory.
  // It would probably be nicer to use DMA mode in the future, but the
  // documentation for DMA functionality is somewhat limited and I haven't
  // spent much time playing around with it.

  DBG_ISR_LOG("todo: receive OUT packet");
#if 0
  auto *xfer = &xfer_status_[endpoint_num].out;
  // All RX transfers are performed using FIFO 0.
  volatile uint32_t *rx_fifo = usb_->fifo[0];

  // We can read up to the smaller of:
  // - the remaining size until the buffer is full
  // - the maximum transfer size for this endpoint
  // - or the packet_size argument
  //
  // TODO: If bufsize_left is less than packet_size, the host may have sent a
  // full packet but we are not able to read all of it.  I haven't played
  // around with how the ESP32 behaves in this case, but I think it will mess
  // up the current code's detection of end of transmission.  We probably
  // should require that start_out_read() always be called with a buffer size
  // that is a multiple of the maximum endpoint packet size.
  const uint16_t bufsize_left = xfer->buffer_size - xfer->received_bytes;
  uint16_t read_size =
      std::min(std::min(bufsize_left, xfer->max_size), packet_size);

  uint8_t *out_ptr = (xfer->buffer + xfer->received_bytes);
  uint8_t *const end = out_ptr + read_size;
  // Read 32-bit words at a time from the FIFO
  // Copy into out_ptr with memcmp, since out_ptr may not be word-aligned.
  while (out_ptr + 4 < end) {
    const uint32_t tmp = (*rx_fifo);
    memcpy(out_ptr, &tmp, 4);
    out_ptr += 4;
  }
  if (out_ptr < end) {
    const uint32_t tmp = (*rx_fifo);
    memcpy(out_ptr, &tmp, end - out_ptr);
  }

  xfer->received_bytes = end - xfer->buffer;

  // An OUT packet with a length less than the maximum endpoint packet size
  // always indicates the end of a transfer.
  xfer->recv_complete = (packet_size < xfer->max_size) ||
                        (xfer->received_bytes == xfer->buffer_size);
#endif
}

} // namespace ausb
