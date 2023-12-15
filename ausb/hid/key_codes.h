// Copyright (c) 2023, Adam Simpkins
#pragma once

#include "ausb/hid/usage_page.h"

namespace ausb::hid {

/**
 * Usage Codes for the Key Codes Usage Page (0x07)
 *
 * Defined in the HID Usage Tables v1.3 spec:
 * https://usb.org/sites/default/files/hut1_3_0.pdf
 * (Section 10, page 88)
 */
enum class Key : uint8_t {
  None = 0x00, // reserved
  ErrorRollOver = 0x01,
  POSTFail = 0x02,
  ErrorUndefined = 0x03,
  A = 0x04,
  B = 0x05,
  C = 0x06,
  D = 0x07,
  E = 0x08,
  F = 0x09,
  G = 0x0a,
  H = 0x0b,
  I = 0x0c,
  J = 0x0d,
  K = 0x0e,
  L = 0x0f,
  M = 0x10,
  N = 0x11,
  O = 0x12,
  P = 0x13,
  Q = 0x14,
  R = 0x15,
  S = 0x16,
  T = 0x17,
  U = 0x18,
  V = 0x19,
  W = 0x1a,
  X = 0x1b,
  Y = 0x1c,
  Z = 0x1d,
  Num1 = 0x1e,
  Num2 = 0x1f,
  Num3 = 0x20,
  Num4 = 0x21,
  Num5 = 0x22,
  Num6 = 0x23,
  Num7 = 0x24,
  Num8 = 0x25,
  Num9 = 0x26,
  Num0 = 0x27,
  Enter = 0x28,
  Escape = 0x29,
  Backspace = 0x2a,
  Tab = 0x2b,
  Space = 0x2c,
  Minus = 0x2d,
  Equal = 0x2e,
  BracketLeft = 0x2f,
  BracketRight = 0x30,
  Backslash = 0x31,
  NonUSNumber = 0x32,
  Semicolon = 0x33,
  Quote = 0x34,
  Tilde = 0x35,
  Comma = 0x36,
  Period = 0x37,
  Slash = 0x38,
  CapsLock = 0x39,
  F1 = 0x3a,
  F2 = 0x3b,
  F3 = 0x3c,
  F4 = 0x3d,
  F5 = 0x3e,
  F6 = 0x3f,
  F7 = 0x40,
  F8 = 0x41,
  F9 = 0x42,
  F10 = 0x43,
  F11 = 0x44,
  F12 = 0x45,
  PrintScreen = 0x46,
  ScrollLock = 0x47,
  Pause = 0x48,
  Insert = 0x49,
  Home = 0x4a,
  PageUp = 0x4b,
  Delete = 0x4c,
  End = 0x4d,
  PageDown = 0x4e,
  Right = 0x4f,
  Left = 0x50,
  Down = 0x51,
  Up = 0x52,
  NumLock = 0x53,
  KeypadDivide = 0x54,
  KeypadMultiply = 0x55,
  KeypadMinus = 0x56,
  KeypadPlus = 0x57,
  KeypadEnter = 0x58,
  Keypad1 = 0x59,
  Keypad2 = 0x5a,
  Keypad3 = 0x5b,
  Keypad4 = 0x5c,
  Keypad5 = 0x5d,
  Keypad6 = 0x5e,
  Keypad7 = 0x5f,
  Keypad8 = 0x60,
  Keypad9 = 0x61,
  Keypad0 = 0x62,
  KeypadPeriod = 0x63,
  NonUSBackslash = 0x64,
  Application = 0x65,
  Power = 0x66,
  KeypadEqual = 0x67,
  F13 = 0x68,
  F14 = 0x69,
  F15 = 0x6a,
  F16 = 0x6b,
  F17 = 0x6c,
  F18 = 0x6d,
  F19 = 0x6e,
  F20 = 0x6f,
  F21 = 0x70,
  F22 = 0x71,
  F23 = 0x72,
  F24 = 0x73,
  Execute = 0x74,
  Help = 0x75,
  Menu = 0x76,
  Select = 0x77,
  Stop = 0x78,
  Again = 0x79,
  Undo = 0x7a,
  Cut = 0x7b,
  Copy = 0x7c,
  Paste = 0x7d,
  Find = 0x7e,
  Mute = 0x7f,
  VolumeUp = 0x80,
  VolumeDown = 0x81,
  LockingCapsLock = 0x82,
  LockingNumLock = 0x83,
  LockingScrollLock = 0x84,
  KeypadComma = 0x85,
  KeypadEqualSign = 0x86, // rare; KeypadEqual is normally what you want
  International1 = 0x87,
  International2 = 0x88,
  International3 = 0x89,
  International4 = 0x8a,
  International5 = 0x8b,
  International6 = 0x8c,
  International7 = 0x8d,
  International8 = 0x8e,
  International9 = 0x8f,
  Lang1 = 0x90, // Hangul/English
  Lang2 = 0x91, // Hanja conversion
  Lang3 = 0x92, // Katakana
  Lang4 = 0x93, // Hiragana
  Lang5 = 0x94, // Zenkaku/Hankaku
  Lang6 = 0x95,
  Lang7 = 0x96,
  Lang8 = 0x97,
  Lang9 = 0x98,
  AlternateErase = 0x99,
  SysReq = 0x9a,
  Cancel = 0x9b,
  Clear = 0x9c,
  Prior = 0x9d,
  Return = 0x9e, // rare; Enter is normally what you want
  Separator = 0x9f,
  Out = 0xa0,
  Oper = 0xa1,
  ClearAgain = 0xa2,
  CrSelProps = 0xa3,
  ExSel = 0xa4,
  Keypad00 = 0xb0,
  Keypad000 = 0xb1,
  ThousandsSeparator = 0xb2,
  DecimalSeparator = 0xb3,
  CurrencyUnit = 0xb4,
  CurrencySubUnit = 0xb5,
  KeypadOpenParen = 0xb6,
  KeypadCloseParen = 0xb7,
  KeypadOpenBrace = 0xb8,
  KeypadCloseBrace = 0xb9,
  KeypadTab = 0xba,
  KeypadBackspace = 0xbb,
  KeypadA = 0xbc,
  KeypadB = 0xbd,
  KeypadC = 0xbe,
  KeypadD = 0xbf,
  KeypadE = 0xc0,
  KeypadF = 0xc1,
  KeypadXOR = 0xc2,
  KeypadCaret = 0xc3,
  KeypadPercent = 0xc4,
  KeypadLeftAngle = 0xc5,
  KeypadRightAngle = 0xc6,
  KeypadAnd = 0xc7,
  KeypadAndAnd = 0xc8,
  KeypadOr = 0xc9,
  KeypadOrOr = 0xca,
  KeypadColon = 0xcb,
  KeypadHash = 0xcc,
  KeypadSpace = 0xcd,
  KeypadAt = 0xce,
  KeypadExclamation = 0xcf,
  KeypadMemoryStore = 0xd0,
  KeypadMemoryRecall = 0xd1,
  KeypadMemoryClear = 0xd2,
  KeypadMemoryAdd = 0xd3,
  KeypadMemorySubtract = 0xd4,
  KeypadMemoryMultiply = 0xd5,
  KeypadMemoryDivide = 0xd6,
  KeypadMemoryPlusMinus = 0xd7,
  KeypadClear = 0xd8,
  KeypadClearEntry = 0xd9,
  KeypadBinary = 0xda,
  KeypadOctal = 0xdb,
  KeypadHexadecimal = 0xdc,
  LeftControl = 0xe0,
  LeftShift = 0xe1,
  LeftAlt = 0xe2,
  LeftGui = 0xe3,
  RightControl = 0xe4,
  RightShift = 0xe5,
  RightAlt = 0xe6,
  RightGui = 0xe7,
};
template <>
class is_usage_type<Key> : public std::true_type {};

enum class Modifier : uint8_t {
  None = 0x00,
  Control = 0x01,
  Shift = 0x02,
  Alt = 0x04,
  Gui = 0x08,
  LeftControl = 0x01,
  LeftShift = 0x02,
  LeftAlt = 0x04,
  LeftGui = 0x08,
  RightControl = 0x10,
  RightShift = 0x20,
  RightAlt = 0x40,
  RightGui = 0x80,
};

} // namespace ausb::hid
