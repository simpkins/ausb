// Copyright (c) 2023, Adam Simpkins
#include "ausb/device/MessagePipe.h"

#include "ausb/SetupPacket.h"
#include "ausb/device/ControlMessageHandler.h"
#include "ausb/device/CtrlInXfer.h"
#include "ausb/device/CtrlOutXfer.h"
#include "ausb/device/EndpointManager.h"
#include "ausb/log.h"

#include <cassert>

namespace ausb::device {

MessagePipe::~MessagePipe() {
  on_unconfigured(XferFailReason::LocalReset);
}

void MessagePipe::on_unconfigured(XferFailReason reason) {
  if (status_ != Status::Idle) {
    invoke_xfer_failed(reason);
  }
}

void MessagePipe::on_setup_received(const SetupPacket &packet) {
  AUSB_LOGI("SETUP received: request_type=0x%02x request=0x%02x "
            "value=0x%04x index=0x%04x length=0x%04x",
            packet.request_type,
            packet.request,
            packet.value,
            packet.index,
            packet.length);

  // Handle receipt of a new SETUP packet if we think that we are currently
  // still processing an existing control transfer.
  if (status_ != Status::Idle) [[unlikely]] {
    // One possible cause for this is simply from the host retransmitting a
    // SETUP packet after it thought there was a transmission error.  If this
    // packet is identical to the SETUP packet we are currently processing and
    // we haven't seen any IN or OUT tokens since that SETUP packet, this is
    // just a retransmission and we can ignore it.
    //
    // Different hardware implementations provide different functionality for
    // filtering out or detecting retransmitted SETUP packets.
    if (setup_rxmit_detector_.is_retransmit(packet)) {
      AUSB_LOGI("ignoring retransmitted SETUP packet");
      return;
    }

    // Otherwise this generally indicates a bug somewhere.
    //
    // The most likely cause is probably a bug in the device implementation.
    // e.g., this can happen if a CtrlInXfer implementation attempts to send a
    // different amount of data than specified in the setup packet wLength
    // field, or similarly if a CtrlOutXfer attempts to read the wrong length.
    //
    // This also could be caused by a badly behaving host, but if you are
    // hitting this it's more likely that you are implementing a new USB device
    // and just have a bug in your device behavior somewhere.
    AUSB_LOGE("received SETUP packet when existing control transfer is in "
              "progress: state=%d",
              static_cast<int>(status_));
    fail_current_xfer(XferFailReason::ProtocolError);
    return;
  }

  // Process the SETUP packet
  setup_rxmit_detector_.on_setup(packet);
  if (packet.get_direction() == Direction::Out) {
    status_ = Status::OutXfer;
    xfer_.out = handler_->process_out_setup(this, packet);
    if (xfer_.out) {
      xfer_.out->start(packet);
    } else {
      AUSB_LOGE(
          "unhandled SETUP OUT packet: request_type=0x%02x request=0x%02x "
          "value=0x%04x index=0x%04x length=0x%04x",
          packet.request_type,
          packet.request,
          packet.value,
          packet.index,
          packet.length);
      status_ = Status::Idle;
      manager_->stall_message_pipe(endpoint_num_);
    }
  } else {
    status_ = Status::InSetupReceived;
    xfer_.in = handler_->process_in_setup(this, packet);
    if (xfer_.in) {
      xfer_.in->start(packet);
    } else {
      AUSB_LOGE("unhandled SETUP IN packet: request_type=0x%02x request=0x%02x "
                "value=0x%04x index=0x%04x length=0x%04x",
                packet.request_type,
                packet.request,
                packet.value,
                packet.index,
                packet.length);
      status_ = Status::Idle;
      manager_->stall_message_pipe(endpoint_num_);
    }
  }
}

void MessagePipe::on_in_ep_unconfigured(XferFailReason reason) {
  // We generally expect on_in_ep_unconfigured() and on_out_ep_unconfigured()
  // to be called together.  It's fine for us to call on_unconfigured() twice
  // in a row, so just let each call run on_unconfigured().
  on_unconfigured(reason);
}

void MessagePipe::on_out_ep_unconfigured(XferFailReason reason) {
  on_unconfigured(reason);
}

void MessagePipe::on_in_xfer_complete() {
  switch (status_) {
  case Status::InSendPartial:
    xfer_.in->partial_write_complete();
    return;
  case Status::InSendFinal:
    AUSB_LOGD("control IN write complete");
    status_ = Status::InStatus;
    // Wait for the host to acknowledge our data with a 0-length OUT packet.
    manager_->start_ctrl_in_ack(this);
    return;
  case Status::OutAck:
    AUSB_LOGD("control OUT status complete");
    xfer_.out->ack_complete();
    destroy_out_xfer();
    return;
  case Status::Idle:
  case Status::OutXfer:
  case Status::InSetupReceived:
  case Status::InStatus:
    // Continue to failure handling below
    break;
  }

  AUSB_LOGE("control EP%u received IN xfer complete in unexpected state %d",
            endpoint_num_,
            static_cast<int>(status_));
  fail_current_xfer(XferFailReason::SoftwareError);
}

void MessagePipe::on_in_xfer_failed(XferFailReason reason) {
  // This should only occur after we started an IN operation, which is either
  // in Status::InSendPartial, Status::InSendFinal, or Status::OutAck.
  AUSB_LOGW("control IN failure: status=%d, reason=%d",
            static_cast<int>(status_),
            static_cast<int>(reason));

  invoke_xfer_failed(reason);
  manager_->stall_message_pipe(endpoint_num_);
}

void MessagePipe::on_out_xfer_complete(uint32_t bytes_read) {
  switch (status_) {
  case Status::OutXfer:
    xfer_.out->out_data_received(bytes_read);
    return;
  case Status::InStatus:
    AUSB_LOGD("control IN status successfully ACKed");
    xfer_.in->xfer_acked();
    destroy_in_xfer();
    return;
  case Status::Idle:
  case Status::OutAck:
  case Status::InSetupReceived:
  case Status::InSendPartial:
  case Status::InSendFinal:
    // Continue to failure handling below
    break;
  }
  AUSB_LOGE("control EP%u received OUT xfer complete in unexpected state %d",
            endpoint_num_,
            static_cast<int>(status_));
  fail_current_xfer(XferFailReason::SoftwareError);
}

void MessagePipe::on_out_xfer_failed(XferFailReason reason) {
  // This should only occur after we started an IN operation, which is either
  // in Status::InStatus, or Status::OutXfer.
  AUSB_LOGW("control OUT failure: status=%d, reason=%d",
            static_cast<int>(status_),
            static_cast<int>(reason));

  invoke_xfer_failed(reason);
  manager_->stall_message_pipe(endpoint_num_);
}

CtrlOutXfer *MessagePipe::process_out_setup(MessagePipe *pipe,
                               const SetupPacket &packet) {
  // In theory this could be called if the host sends a SETUP message
  // with an endpoint recipient of 0.  We generally don't expect this, though.
  // Return null, and let our code above log a message about this unhandled
  // transfer.
  return nullptr;
}

CtrlInXfer *MessagePipe::process_in_setup(MessagePipe *pipe,
                             const SetupPacket &packet) {
  return nullptr;
}

void MessagePipe::start_out_read(void *data, size_t size) {
  if (status_ != Status::OutXfer) [[unlikely]] {
    AUSB_LOGE("start_out_read() called in unexpected state %d",
              static_cast<int>(status_));
    fail_current_xfer(XferFailReason::SoftwareError);
    return;
  }

  manager_->start_ctrl_out_read(this, data, size);
}

void MessagePipe::ack_out_xfer() {
  if (status_ != Status::OutXfer) [[unlikely]] {
    AUSB_LOGE("ack_out_xfer() called in unexpected state %d",
              static_cast<int>(status_));
    fail_current_xfer(XferFailReason::SoftwareError);
    return;
  }

  status_ = Status::OutAck;
  manager_->start_ctrl_out_ack(this);
}

void MessagePipe::fail_out_xfer() {
  if (status_ != Status::OutXfer) [[unlikely]] {
    AUSB_LOGE("fail_out_xfer() called in unexpected state %d",
              static_cast<int>(status_));
    fail_current_xfer(XferFailReason::SoftwareError);
    return;
  }

  destroy_out_xfer();
  manager_->stall_message_pipe(endpoint_num_);
}

void MessagePipe::start_in_write(const void *data, size_t size, bool is_final) {
  if (status_ != Status::InSetupReceived && status_ != Status::InSendPartial)
      [[unlikely]] {
    AUSB_LOGE("start_in_write() called in unexpected state %d",
              static_cast<int>(status_));
    fail_current_xfer(XferFailReason::SoftwareError);
    return;
  }

  status_ = is_final ? Status::InSendFinal : Status::InSendPartial;
  manager_->start_ctrl_in_write(this, data, size);
}

void MessagePipe::fail_in_xfer() {
  if (status_ != Status::InSetupReceived && status_ != Status::InSendPartial)
      [[unlikely]] {
    AUSB_LOGE("fail_in_xfer() called in unexpected state %d",
              static_cast<int>(status_));
    fail_current_xfer(XferFailReason::SoftwareError);
    return;
  }

  destroy_in_xfer();
  manager_->stall_message_pipe(endpoint_num_);
}

void MessagePipe::fail_current_xfer(XferFailReason reason) {
  status_ = Status::Idle;
  invoke_xfer_failed(reason);
  manager_->stall_message_pipe(endpoint_num_);
}

void MessagePipe::invoke_xfer_failed(XferFailReason reason) {
  // TODO: invoke_xfer_failed() can potentially be invoked from inside one of
  // the CtrlInXfer or CtrlOutXfer methods.  It would probably be better to
  // avoid immediately destroying the xfer object before we return, and instead
  // defer it's destruction until the start of the next task loop iteration.
  // This would avoid us deleting the xfer object while it may still be running
  // on the stack.  (Deleting it while it is running isn't necessarily a
  // problem, but it is a problem if the running method attempts to access any
  // member variables after we destroy it.)

  switch (status_) {
  case Status::Idle:
    return;
  case Status::OutXfer:
  case Status::OutAck:
    xfer_.out->invoke_xfer_failed(reason);
    destroy_out_xfer();
    return;
  case Status::InSetupReceived:
  case Status::InSendPartial:
  case Status::InSendFinal:
  case Status::InStatus:
    xfer_.in->invoke_xfer_failed(reason);
    destroy_in_xfer();
    return;
  }

  AUSB_LOGE("fail xfer in unknown control endpoint state %d",
            static_cast<int>(status_));
}

void MessagePipe::destroy_in_xfer() {
  assert(status_ == Status::InSetupReceived ||
         status_ == Status::InSendPartial || status_ == Status::InSendFinal ||
         status_ == Status::InStatus);
  delete xfer_.in;
  xfer_.in = nullptr;
  status_ = Status::Idle;
}

void MessagePipe::destroy_out_xfer() {
  assert(status_ == Status::OutXfer || status_ == Status::OutAck);
  delete xfer_.out;
  xfer_.out = nullptr;
  status_ = Status::Idle;
}

} // namespace ausb::device
