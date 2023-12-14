// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/desc/DeviceDescriptor.h"
#include "ausb/dev/CtrlInXfer.h"

#include <asel/buf_view.h>

namespace ausb {

namespace device {

/**
 * A handler for GET_DESCRIPTOR that returns a device descriptor with a
 * modified maximum packet value for endpoint 0.
 *
 * This can be used to modify the device descriptor at runtime if the
 * negotiated endpoint 0 packet size does not match the normal value in the
 * read-only descriptor.  This likely shouldn't be necessary in most cases, and
 * usually the device descriptor can be returned with GetStaticDescriptor
 * instead.
 */
class GetDevDescriptorModifyEP0 : public CtrlInXfer {
public:
  GetDevDescriptorModifyEP0(MessagePipe *ep, asel::buf_view buf,
                            uint8_t correct_ep0_mps);

  void start(const SetupPacket &packet) override;
  void xfer_acked() override;
  void xfer_failed(XferFailReason reason) override;

private:
  DeviceDescriptor desc_;
};

} // namespace device
} // namespace ausb
