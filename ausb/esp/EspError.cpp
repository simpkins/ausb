// Copyright (c) 2023, Adam Simpkins
#include "ausb/esp/EspError.h"

#include <cstring>

namespace ausb {

esp_error_category g_esp_error_category;

std::string esp_error_category::message(int condition) const {
  std::string msg;
  msg.resize(32);
  esp_err_to_name_r(condition, msg.data(), msg.size());
  msg.resize(strnlen(msg.data(), msg.size()));
  return msg;
}

} // namespace ausb
