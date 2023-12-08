// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/usb_types.h"

#include <cstdint>
#include <cstdlib>
#include <type_traits>

namespace ausb {

enum class DescriptorType : uint8_t {
  Device = 1,
  Config = 2,
  String = 3,
  Interface = 4,
  Endpoint = 5,
  DeviceQualifier = 6,
  OtherSpeedConfig = 7,
  InterfacePower = 8,
  Hid = 0x21,
  HidReport = 0x22,
  HidPhysical = 0x23,
};

/**
 * Create the setup wValue field for a given descriptor type and descriptor
 * index.
 */
constexpr uint16_t desc_setup_value(DescriptorType type, uint8_t index = 0) {
  return (static_cast<uint16_t>(type) << 8) | index;
}

enum class UsbClass : uint8_t {
  PerInterface = 0x00,
  Audio = 0x01,
  Cdc = 0x02,
  Hid = 0x03,
  Physical = 0x05,
  Image = 0x06,
  Printer = 0x07,
  MassStorage = 0x08,
  Hub = 0x09,
  CdcData = 0x0a,
  SmartCard = 0x0b,
  ContentSecurity = 0xd,
  Video = 0x0e,
  PersonalHealthcare = 0x0f,
  AudioVideo = 0x10,
  Billboard = 0x11,
  UsbCBridge = 0x12,
  I3C = 0x3c,
  Diagnostic = 0xdc,
  WirelessController = 0xe0,
  Miscellaneous = 0xef,
  VendorSpecific = 0xff,
};

enum class FeatureSelector {
  EndpointHalt = 0, // Only valid for SetupRecipient::Endpoint
  RemoteWakeup = 1, // Only valid for SetupRecipient::Device
  TestMode = 2,     // Only valid for SetupRecipient::Device
};

enum class EndpointSync : uint8_t {
  NoSync = 0,
  Async = 1 << 2,
  Adaptive = 2 << 2,
  Sync = 3 << 2,
};

enum class EndpointUsage : uint8_t {
  Data = 0,
  Feedback = 1 << 4,
  ImplicitFeedback = 2 << 4,
  // (3 << 4) is reserved
};

/**
 * Attribute bits for the bmAttributes field in the config descriptor
 */
enum class ConfigAttr : uint8_t {
  None = 0x00,
  RemoteWakeup = 0x20,
  SelfPowered = 0x40,
};

inline constexpr ConfigAttr operator|(ConfigAttr a1, ConfigAttr a2) {
  return static_cast<ConfigAttr>(static_cast<uint8_t>(a1) |
                                 static_cast<uint8_t>(a2));
}

inline constexpr ConfigAttr operator&(ConfigAttr a1, ConfigAttr a2) {
  return static_cast<ConfigAttr>(static_cast<uint8_t>(a1) &
                                 static_cast<uint8_t>(a2));
}

inline constexpr ConfigAttr& operator|=(ConfigAttr& a1, ConfigAttr a2) {
  a1 = a1 | a2;
  return a1;
}

/**
 * A helper class for the max_power field in the config descriptor.
 *
 * This value tracks power in units of 2 milliamps.
 */
class UsbMilliamps {
public:
  explicit constexpr UsbMilliamps(uint16_t milliamps)
      : value_((milliamps + 1) / 2) {
    if (milliamps >= (0xff * 2)) {
      if (std::is_constant_evaluated()) {
        // We can generate a compile failure if called with bad data at compile
        // time
        abort(); // "value too large to express"
      } else {
        // If invoked with bad data at runtime, clamp the value to the max
        value_ = 0xff;
      }
    }
  }

  constexpr uint8_t value_in_2ma() const { return value_; }
  constexpr uint16_t milliamps() const {
    return static_cast<uint16_t>(value_) * 2;
  }

private:
  uint8_t value_{0};
};

class EndpointAttributes {
public:
  explicit constexpr EndpointAttributes(EndpointType type)
      : value_{static_cast<uint8_t>(type)} {}

  explicit constexpr EndpointAttributes(EndpointType type, EndpointUsage usage)
      : value_{static_cast<uint8_t>(static_cast<uint8_t>(type) |
                                    static_cast<uint8_t>(usage))} {}

  explicit constexpr EndpointAttributes(EndpointSync sync, EndpointUsage usage)
      : value_{static_cast<uint8_t>(
            static_cast<uint8_t>(EndpointType::Isochronous) |
            static_cast<uint8_t>(sync) | static_cast<uint8_t>(usage))} {}

  constexpr EndpointType type() const {
    return EndpointType{static_cast<uint8_t>(value_ & 0x03)};
  }
  constexpr EndpointSync sync_type() const {
    return EndpointSync{static_cast<uint8_t>(value_ & (0x03 << 2))};
  }
  constexpr EndpointUsage usage() const {
    return EndpointUsage{static_cast<uint8_t>(value_ & (0x03 << 4))};
  }

  constexpr uint8_t value() const {
    return value_;
  }

private:
  uint8_t value_;
};

enum class InterfaceClass : uint8_t {
  Hid = 3,
};

enum class Language : uint16_t {
  Afrikaans = 0x0436,
  Albanian = 0x041c,
  Arabic_Saudi_Arabia = 0x0401,
  Arabic_Iraq = 0x0801,
  Arabic_Egypt = 0x0c01,
  Arabic_Libya = 0x1001,
  Arabic_Algeria = 0x1401,
  Arabic_Morocco = 0x1801,
  Arabic_Tunisia = 0x1c01,
  Arabic_Oman = 0x2001,
  Arabic_Yemen = 0x2401,
  Arabic_Syria = 0x2801,
  Arabic_Jordan = 0x2c01,
  Arabic_Lebanon = 0x3001,
  Arabic_Kuwait = 0x3401,
  Arabic_UAE = 0x3801,
  Arabic_Bahrain = 0x3c01,
  Arabic_Qatar = 0x4001,
  Armenian = 0x042b,
  Assamese = 0x044d,
  Azeri_Latin = 0x042c,
  Azeri_Cyrillic = 0x082c,
  Basque = 0x042d,
  Belarussian = 0x0423,
  Bengali = 0x0445,
  Bulgarian = 0x0402,
  Burmese = 0x0455,
  Catalan = 0x0403,
  Chinese_Taiwan = 0x0404,
  Chinese_PRC = 0x0804,
  Chinese_Hong_Kong = 0x0c04,
  Chinese_Singapore = 0x1004,
  Chinese_Macau = 0x1404,
  Croatian = 0x041a,
  Czech = 0x0405,
  Danish = 0x0406,
  Dutch_Netherlands = 0x0413,
  Dutch_Belgium = 0x0813,
  English_US = 0x0409,
  English_UK = 0x0809,
  English_Australian = 0x0c09,
  English_Canadian = 0x1009,
  English_New_Zealand = 0x1409,
  English_Ireland = 0x1809,
  English_South_Africa = 0x1c09,
  English_Jamaica = 0x2009,
  English_Caribbean = 0x2409,
  English_Belize = 0x2809,
  English_Trinidad = 0x2c09,
  English_Zimbabwe = 0x3009,
  English_Philippines = 0x3409,
  Estonian = 0x0425,
  Faeroese = 0x0438,
  Farsi = 0x0429,
  Finnish = 0x040b,
  French_Standard = 0x040c,
  French_Belgian = 0x080c,
  French_Canadian = 0x0c0c,
  French_Switzerland = 0x100c,
  French_Luxembourg = 0x140c,
  French_Monaco = 0x180c,
  Georgian = 0x0437,
  German_Standard = 0x0407,
  German_Switzerland = 0x0807,
  German_Austria = 0x0c07,
  German_Luxembourg = 0x1007,
  German_Liechtenstein = 0x1407,
  Greek = 0x0408,
  Gujarati = 0x0447,
  Hebrew = 0x040d,
  Hindi = 0x0439,
  Hungarian = 0x040e,
  Icelandic = 0x040f,
  Indonesian = 0x0421,
  Italian_Standard = 0x0410,
  Italian_Switzerland = 0x0810,
  Japanese = 0x0411,
  Kannada = 0x044b,
  Kashmiri_India = 0x0860,
  Kazakh = 0x043f,
  Konkani = 0x0457,
  Korean = 0x0412,
  Korean_Johab = 0x0812,
  Latvian = 0x0426,
  Lithuanian = 0x0427,
  Lithuanian_Classic = 0x0827,
  Macedonian = 0x042f,
  Malay_Malaysian = 0x043e,
  Malay_Brunei_Darussalam = 0x083e,
  Malayalam = 0x044c,
  Manipuri = 0x0458,
  Marathi = 0x044e,
  Nepali_India = 0x0861,
  Norwegian_Bokmal = 0x0414,
  Norwegian_Nynorsk = 0x0814,
  Oriya = 0x0448,
  Polish = 0x0415,
  Portuguese_Brazil = 0x0416,
  Portuguese_Standard = 0x0816,
  Punjabi = 0x0446,
  Romanian = 0x0418,
  Russian = 0x0419,
  Sanskrit = 0x044f,
  Serbian_Cyrillic = 0x0c1a,
  Serbian_Latin = 0x081a,
  Sindhi = 0x0459,
  Slovak = 0x041b,
  Slovenian = 0x0424,
  Spanish_Traditional_Sort = 0x040a,
  Spanish_Mexican = 0x080a,
  Spanish_Modern_Sort = 0x0c0a,
  Spanish_Guatemala = 0x100a,
  Spanish_Costa_Rica = 0x140a,
  Spanish_Panama = 0x180a,
  Spanish_Dominican_Republic = 0x1c0a,
  Spanish_Venezuela = 0x200a,
  Spanish_Colombia = 0x240a,
  Spanish_Peru = 0x280a,
  Spanish_Argentina = 0x2c0a,
  Spanish_Ecuador = 0x300a,
  Spanish_Chile = 0x340a,
  Spanish_Uruguay = 0x380a,
  Spanish_Paraguay = 0x3c0a,
  Spanish_Bolivia = 0x400a,
  Spanish_El_Salvador = 0x440a,
  Spanish_Honduras = 0x480a,
  Spanish_Nicaragua = 0x4c0a,
  Spanish_Puerto_Rico = 0x500a,
  Sutu = 0x0430,
  Swahili_Kenya = 0x0441,
  Swedish = 0x041d,
  Swedish_Finland = 0x081d,
  Tamil = 0x0449,
  Tatar_Tatarstan = 0x0444,
  Telugu = 0x044a,
  Thai = 0x041e,
  Turkish = 0x041f,
  Ukrainian = 0x0422,
  Urdu_Pakistan = 0x0420,
  Urdu_India = 0x0820,
  Uzbek_Latin = 0x0443,
  Uzbek_Cyrillic = 0x0843,
  Vietnamese = 0x042a,
  HID_Usage_Data_Descriptor = 0x04ff,
  HID_Vendor_Defined_1 = 0xf0ff,
  HID_Vendor_Defined_2 = 0xf4ff,
  HID_Vendor_Defined_3 = 0xf8ff,
  HID_Vendor_Defined_4 = 0xfcff,
};

/**
 * Create the setup wIndex field for a given descriptor language.
 */
constexpr uint16_t desc_setup_index(Language lang) {
  return static_cast<uint16_t>(lang);
}

} // namespace ausb
