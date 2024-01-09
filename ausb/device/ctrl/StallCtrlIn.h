// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/device/CtrlInXfer.h"

namespace ausb::device {

/**
 * A CtrlInXfer handler that simply fails the request with a STALL error.
 *
 * Returning a null CtrlInXfer object from process_in_setup() would also stall
 * the request, however this also logs an error about the request being
 * unhandled.  The StallCtrlIn object helps make it clear that we are
 * intentionally failing this request, rather than forgetting/being unable to
 * handle it.
 */
class StallCtrlIn : public CtrlInXfer {
public:
  using CtrlInXfer::CtrlInXfer;

  void start(const SetupPacket &packet) override;
  void xfer_acked() override;
  void xfer_failed(XferFailReason reason) override;
};

} // namespace ausb::device
