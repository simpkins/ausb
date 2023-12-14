cmake_minimum_required(VERSION 3.19)

set(CMAKE_CXX_STANDARD 20)

set(AUSB_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})
set(
    srcs
    "ausb/desc/StaticDescriptorMapUtils.cpp"
    "ausb/dev/CtrlInXfer.cpp"
    "ausb/dev/CtrlOutXfer.cpp"
    "ausb/dev/EndpointManager.cpp"
    "ausb/dev/EndpointZero.cpp"
    "ausb/dev/MessagePipe.cpp"
    "ausb/dev/StdControlHandler.cpp"
    "ausb/dev/ctrl/AckEmptyCtrlOut.cpp"
    "ausb/dev/ctrl/GetDevDescriptorModifyEP0.cpp"
    "ausb/dev/ctrl/GetStaticDescriptor.cpp"
    "ausb/dev/ctrl/SetAddress.cpp"
    "ausb/dev/ctrl/StallCtrlIn.cpp"
    "ausb/dev/ctrl/StallCtrlOut.cpp"
    "ausb/hid/KeyboardInterface.cpp"
    "ausb/hid/HidSetReport.cpp"
    "ausb/log.cpp"
)

if(ESP_PLATFORM)
    # We are being invoked as part of ESP-IDF's custom CMake toolchain.
    # Use idf_component_register() rather than normal CMake project setup.
    include("${CMAKE_CURRENT_LIST_DIR}/cmake/esp.cmake")
else()
    project(ausb HOMEPAGE_URL https://github.com/simpkins/ausb)

    include("${CMAKE_CURRENT_LIST_DIR}/cmake/compiler-settings.cmake")
    add_subdirectory("test")

    add_subdirectory("asel")

    set(
        AUSB_CONFIG_HW_MULTI
        OFF CACHE BOOL
        "Explicitly force runtime support for multiple hardware implementations"
    )
    mark_as_advanced(AUSB_CONFIG_HW_MULTI)

    set(AUSB_CONFIG_HW_MOCK ON)
    set(hw_srcs "ausb/hw/mock/MockDevice.cpp")

    add_library(ausb STATIC ${srcs} ${hw_srcs})
    target_include_directories(
        ausb PUBLIC
        "${AUSB_ROOT_DIR}"
        "${CMAKE_CURRENT_BINARY_DIR}"
    )
    target_link_libraries(
        ausb PUBLIC
        asel
    )

    add_executable(ausb_tests test/runner/mock/main.cpp)
    target_link_libraries(ausb_tests PUBLIC ausb_test_cases)

    if(NOT CMAKE_CROSSCOMPILING)
        include(CTest)
        add_test(NAME ausb COMMAND ausb_tests)
    endif()

    configure_file(
        "${AUSB_ROOT_DIR}/ausb/ausb_config_vars.h.in"
        "${CMAKE_CURRENT_BINARY_DIR}/ausb/ausb_config_vars.h"
    )
endif()
