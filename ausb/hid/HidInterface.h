// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/ausb_types.h"
#include "ausb/dev/Interface.h"

namespace ausb::hid {

class HidInterface : public device::Interface {
public:
  [[nodiscard]] virtual bool set_report(asel::buf_view data) = 0;
};

} // namespace ausb::hid
