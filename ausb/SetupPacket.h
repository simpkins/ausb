// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/usb_types.h"

namespace ausb {

enum class SetupRecipient : uint8_t {
  Device = 0,
  Interface = 1,
  Endpoint = 2,
  Other = 3,
};

enum class SetupReqType : uint8_t {
  Standard = 0x00,
  Class = 0x20,
  Vendor = 0x40,
  Reserved = 0x60,
};

enum class StdRequestType : uint8_t {
  GetStatus = 0,
  ClearFeature = 1,
  SetFeature = 3,
  SetAddress = 5,
  GetDescriptor = 6,
  GetConfiguration = 8,
  SetConfiguration = 9,
  GetInterface = 10,
  SetInterface = 11,
};

struct SetupPacket {
  SetupReqType get_request_type() const {
    static constexpr uint8_t kSetupReqTypeMask = 0x60;
    return static_cast<SetupReqType>(request_type & kSetupReqTypeMask);
  }
  Direction get_direction() const {
    return (request_type & 0x80) ? Direction::In : Direction::Out;
  }
  SetupRecipient get_recipient() const {
    return static_cast<SetupRecipient>(request_type & 0x1f);
  }

  // Should only be called if get_request_type() return s
  // SetupReqType::Standard
  StdRequestType get_std_request() const {
    return static_cast<StdRequestType>(request);
  }

  bool operator==(const SetupPacket&) const = default;

  uint8_t request_type = 0;
  uint8_t request = 0;
  uint16_t value = 0;
  uint16_t index = 0;
  uint16_t length = 0;
};

} // namespace ausb
