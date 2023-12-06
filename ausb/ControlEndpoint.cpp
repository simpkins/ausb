// Copyright (c) 2023, Adam Simpkins
#include "ausb/ControlEndpoint.h"

#include "ausb/CtrlInXfer.h"
#include "ausb/CtrlOutXfer.h"
#include "ausb/SetupPacket.h"
#include "ausb/UsbDevice.h"
#include "ausb/log.h"

namespace ausb::device {

ControlEndpoint::~ControlEndpoint() = default;

void ControlEndpoint::on_reset(XferFailReason reason) {
  status_ = Status::Idle;
  invoke_xfer_failed(reason);
  // We don't stall the endpoint here.  The reset itself should flush
  // all transfers from the hardware and disconnect the bus.

  callback_->on_reset(reason);
}

void ControlEndpoint::on_enum_done(uint8_t max_packet_size) {
  callback_->on_enum_done(max_packet_size);
}

void ControlEndpoint::on_setup_received(const SetupPacket &packet) {
  AUSB_LOGI("SETUP received: request_type=0x%02x request=0x%02x "
            "value=0x%04x index=0x%04x length=0x%04x",
            packet.request_type, packet.request, packet.value, packet.index,
            packet.length);

  // Handle receipt of a new SETUP packet if we think that we are currently
  // still processing an existing control transfer.
  if (status_ != Status::Idle) [[unlikely]] {
    // Check to see if we think this is just a retransmitted SETUP packet
    // from the host.
    if (setup_rxmit_detector_.is_retransmit(packet)) {
      AUSB_LOGI("ignoring retransmitted SETUP packet");
      return;
    }

    // If this is not a retransmit, abort the current transfer and respond to
    // this SETUP packet with an error.  (We don't want to attempt to process
    // this transfer, since the previous transfer we thought was in progress
    // might have already given data to the hardware to transmit, and it might
    // have already transmitted it, which the host may misinterpret as
    // belonging to this transfer.)
    AUSB_LOGE("received SETUP packet when existing control transfer is in "
              "progress: state=%d",
              static_cast<int>(status_));
    invoke_xfer_failed(XferFailReason::ProtocolError);
    return;
  }

  // Process the SETUP packet
  setup_rxmit_detector_.on_setup(packet);
  if (packet.get_direction() == Direction::Out) {
    status_ = Status::OutXfer;
    new (&xfer_.in)
        std::unique_ptr<CtrlOutXfer>(callback_->process_out_setup(packet));
    if (xfer_.out) {
      xfer_.out->start(packet);
    } else {
      status_ = Status::Idle;
      xfer_.out.~unique_ptr();
      usb_->stall_control_endpoint(endpoint_num_);
    }
  } else {
    status_ = Status::InSetupReceived;
    new (&xfer_.in)
        std::unique_ptr<CtrlInXfer>(callback_->process_in_setup(packet));
    if (xfer_.in) {
      xfer_.in->start(packet);
    } else {
      status_ = Status::Idle;
      xfer_.in.~unique_ptr();
      usb_->stall_control_endpoint(endpoint_num_);
    }
  }
}

void ControlEndpoint::on_in_xfer_complete() {
  switch (status_) {
  case Status::InSendData:
    AUSB_LOGD("control IN write complete");
    // TODO: If we support partial writes in the future we will need to detect
    // partial vs full write done.  For now we always write all data at once,
    // so once the transfer is finished we move on to the status stage.
    status_ = Status::InStatus;
    // Wait for the host to acknowledge our data with a 0-length OUT packet.
    usb_->start_ctrl_in_ack(this);
    return;
  case Status::OutAck:
    AUSB_LOGD("control OUT status complete");
    extract_out_xfer();
    return;
  case Status::Idle:
  case Status::OutXfer:
  case Status::InSetupReceived:
  case Status::InStatus:
    AUSB_LOGE("on_ep0_in_xfer_complete() received in unexpected state %d",
              static_cast<int>(status_));
    return;
  }

  AUSB_LOGE("unexpected ctrl xfer state %d in on_ep0_in_xfer_complete()",
            static_cast<int>(status_));
}

void ControlEndpoint::on_in_xfer_failed(XferFailReason reason) {
  // This should only occur after we started an IN operation, which is either
  // in Status::InSendData, or Status::OutAck.
  AUSB_LOGW("control IN failure: status=%d, reason%d",
            static_cast<int>(status_), static_cast<int>(reason));

  invoke_xfer_failed(reason);
  usb_->stall_control_endpoint(endpoint_num_);
}

#if 0
void ControlEndpoint::on_out_xfer_complete();
#endif

void ControlEndpoint::on_out_xfer_failed(XferFailReason reason) {
  // This should only occur after we started an IN operation, which is either
  // in Status::InSendData, or Status::OutAck.
  AUSB_LOGW("control OUT failure: status=%d, reason%d",
            static_cast<int>(status_), static_cast<int>(reason));

  invoke_xfer_failed(reason);
  usb_->stall_control_endpoint(endpoint_num_);
}

void ControlEndpoint::ack_out_xfer() {
  if (status_ != Status::OutXfer) {
    AUSB_LOGE("ack_out_xfer() called in unexpected state %d",
              static_cast<int>(status_));
    fail_current_xfer(XferFailReason::SoftwareError);
    return;
  }

  status_ = Status::OutAck;
  usb_->start_ctrl_out_ack(this);
}

void ControlEndpoint::fail_current_xfer(XferFailReason reason) {
  status_ = Status::Idle;
  invoke_xfer_failed(reason);
  usb_->stall_control_endpoint(endpoint_num_);
}

void ControlEndpoint::invoke_xfer_failed(XferFailReason reason) {
  // TODO: invoke_xfer_failed() can potentially be invoked from inside one of
  // the CtrlInXfer or CtrlOutXfer methods.  It would probably be better to
  // avoid immediately destroying the xfer object before we return, and instead
  // defer it's destruction until the start of the next
  // UsbDevice::wait_for_event() call.  This would avoid us deleting the xfer
  // object while it may still be running on the stack.  (Deleting it while it
  // is running isn't necessarily a problem, but it is a problem if the running
  // method attempts to access any member variables after we destroy it.)

  switch (status_) {
  case Status::Idle:
    return;
  case Status::OutXfer:
  case Status::OutAck:
    extract_out_xfer()->invoke_xfer_failed(reason);
    return;
  case Status::InSetupReceived:
  case Status::InSendData:
  case Status::InStatus:
    extract_in_xfer()->invoke_xfer_failed(reason);
    return;
  }

  AUSB_LOGE("fail xfer in unknown control endpoint state %d",
            static_cast<int>(status_));
}

std::unique_ptr<CtrlInXfer> ControlEndpoint::extract_in_xfer() {
  assert(status_ == Status::InSetupReceived || status_ == Status::InSendData ||
         status_ == Status::InStatus);
  std::unique_ptr<CtrlInXfer> result = std::move(xfer_.in);
  status_ = Status::Idle;
  xfer_.in.~unique_ptr();
  return result;
}

std::unique_ptr<CtrlOutXfer> ControlEndpoint::extract_out_xfer() {
  assert(status_ == Status::OutXfer || status_ == Status::OutAck);
  std::unique_ptr<CtrlOutXfer> result = std::move(xfer_.out);
  status_ = Status::Idle;
  xfer_.out.~unique_ptr();
  return result;
}

} // namespace ausb::device
