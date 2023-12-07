cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)

set(
    srcs
    "ausb/desc/StaticDescriptorMapUtils.cpp"
    "ausb/dev/ControlEndpoint.cpp"
    "ausb/dev/ControlHandler.cpp"
    "ausb/dev/CtrlInXfer.cpp"
    "ausb/dev/CtrlOutXfer.cpp"
    "ausb/dev/EndpointManager.cpp"
    "ausb/dev/ctrl/GetDevDescriptorModifyEP0.cpp"
    "ausb/dev/ctrl/GetStaticDescriptor.cpp"
    "ausb/dev/ctrl/SetAddress.cpp"
    "ausb/dev/ctrl/StallCtrlIn.cpp"
)

if(ESP_PLATFORM)
    include("${CMAKE_CURRENT_LIST_DIR}/cmake/esp.cmake")
else()
    include("${CMAKE_CURRENT_LIST_DIR}/cmake/linux.cmake")
endif()
