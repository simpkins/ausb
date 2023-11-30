// Copyright (c) 2023, Adam Simpkins
#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ausb/esp/Esp32Device.h"

extern "C" {

void app_main() {
  while (true) {
    printf("test!\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

} // extern C
