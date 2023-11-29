set(
    srcs
    "ausb/esp/Esp32Device.cpp"
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ".."
    REQUIRED_IDF_TARGETS esp32s2 esp32s3
)
target_compile_definitions(${COMPONENT_LIB} PUBLIC "ESP_TARGET=${target}")
