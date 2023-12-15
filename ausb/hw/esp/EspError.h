// Copyright (c) 2023, Adam Simpkins
#pragma once

#include <esp_err.h>
#include <system_error>

namespace ausb {

class esp_error_category : public std::error_category {
public:
  const char *name() const noexcept override {
    return "esp_error_category";
  }
  std::string message(int condition) const;
};

extern esp_error_category g_esp_error_category;

inline const std::error_category &esp_error() noexcept {
  return g_esp_error_category;
}

inline std::error_code make_esp_error(esp_err_t err) {
  return std::error_code(err, g_esp_error_category);
}

} // namespace ausb
