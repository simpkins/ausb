// Copyright (c) 2022, Adam Simpkins
#pragma once

#include <cstdint>
#include <stdexcept>

namespace ausb {

enum class Direction : uint8_t {
  Out = 0,
  In = 0x80,
};

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

  uint8_t request_type;
  uint8_t request;
  uint16_t value;
  uint16_t index;
  uint16_t length;
};

} // namespace ausb
