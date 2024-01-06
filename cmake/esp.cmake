set(
    esp_srcs
    "ausb/hw/esp/Esp32Device.cpp"
    "ausb/hw/esp/EspError.cpp"
    "ausb/hw/esp/TaskNotificationLoop.cpp"
)
if(CONFIG_AUSB_HW_MOCK)
    list(APPEND esp_srcs "ausb/hw/mock/MockDevice.cpp")
endif()

idf_component_register(
    SRCS ${srcs} ${esp_srcs}
    INCLUDE_DIRS ".."
    REQUIRES "usb"
    REQUIRED_IDF_TARGETS esp32s2 esp32s3
)
target_compile_definitions(${COMPONENT_LIB} PUBLIC "ESP_TARGET=${target}")
target_compile_options(${COMPONENT_LIB} PRIVATE -Werror)
