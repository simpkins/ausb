cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)

set(AUSB_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})
set(
    srcs
    "ausb/desc/StaticDescriptorMapUtils.cpp"
    "ausb/dev/ControlEndpoint.cpp"
    "ausb/dev/ControlHandler.cpp"
    "ausb/dev/CtrlInXfer.cpp"
    "ausb/dev/CtrlOutXfer.cpp"
    "ausb/dev/EndpointManager.cpp"
    "ausb/dev/ctrl/AckEmptyCtrlOut.cpp"
    "ausb/dev/ctrl/GetDevDescriptorModifyEP0.cpp"
    "ausb/dev/ctrl/GetStaticDescriptor.cpp"
    "ausb/dev/ctrl/SetAddress.cpp"
    "ausb/dev/ctrl/StallCtrlIn.cpp"
    "ausb/dev/ctrl/StallCtrlOut.cpp"
    "ausb/hid/KeyboardInterface.cpp"
    "ausb/hid/HidSetReport.cpp"
)

if(ESP_PLATFORM)
    include("${CMAKE_CURRENT_LIST_DIR}/cmake/esp.cmake")
else()
    project(ausb HOMEPAGE_URL https://github.com/simpkins/ausb)

    include("${CMAKE_CURRENT_LIST_DIR}/cmake/linux.cmake")
endif()
