set(
    esp_srcs
    "ausb/esp/Esp32Device.cpp"
    "ausb/esp/EspError.cpp"
)

idf_component_register(
    SRCS ${srcs} ${esp_srcs}
    INCLUDE_DIRS ".."
    REQUIRES "usb"
    REQUIRED_IDF_TARGETS esp32s2 esp32s3
)
target_compile_definitions(${COMPONENT_LIB} PUBLIC "ESP_TARGET=${target}")
