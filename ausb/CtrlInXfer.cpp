// Copyright (c) 2023, Adam Simpkins
#include "ausb/CtrlInXfer.h"

#include "ausb/ControlEndpoint.h"

namespace ausb::device {

void CtrlInXfer::send_partial(const void* data, size_t size) {
  endpoint_->start_in_write(data, size, /*is_final=*/false);
}

void CtrlInXfer::send_final(const void* data, size_t size) {
  endpoint_->start_in_write(data, size, /*is_final=*/true);
}

void CtrlInXfer::error() { endpoint_->fail_in_xfer(); }

} // namespace ausb::device
