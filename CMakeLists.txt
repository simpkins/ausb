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
    if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
        set(AUSB_IS_PRIMARY_PROJECT ON)
    endif()

    include("${CMAKE_CURRENT_LIST_DIR}/cmake/compiler-settings.cmake")

    add_subdirectory("asel")

    set(
        AUSB_CONFIG_HW_MULTI
        OFF CACHE BOOL
        "Explicitly force runtime support for multiple hardware implementations"
    )
    mark_as_advanced(AUSB_CONFIG_HW_MULTI)

    set(AUSB_CONFIG_HW_MOCK ON)
    set(hw_srcs "ausb/hw/mock/MockDevice.cpp")

    # ESP-IDF's Kconfig system defines variables starting with "CONFIG_AUSB_"
    # For use in ausb/ausb-config.h we would rather have variables named
    # AUSB_CONFIG_*.  Translate between the two formats, so that we only have
    # to use one format when checking in our CMake files.  It is easier to
    # convert to the ESP-IDF format here, rather than on the ESP-IDF side,
    # since the ESP-IDF side has multiple components with different entry
    # points.
    set(CONFIG_AUSB_HW_MOCK ${AUSB_CONFIG_HW_MOCK})

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

    if (AUSB_IS_PRIMARY_PROJECT OR AUSB_BUILD_TESTING)
        include(CTest)
        if(BUILD_TESTING)
            add_subdirectory("test")
            if(NOT CMAKE_CROSSCOMPILING)
                # Can't run the tests if cross-compiling
                add_test(NAME ausb COMMAND ausb_tests)
            endif()
        endif()
    endif()

    configure_file(
        "${AUSB_ROOT_DIR}/ausb/ausb_config_vars.h.in"
        "${CMAKE_CURRENT_BINARY_DIR}/ausb/ausb_config_vars.h"
    )
endif()
