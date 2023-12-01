cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)

set(
    srcs
    "ausb/RxBuffer.cpp"
    "ausb/UsbDevice.cpp"
)

if(ESP_PLATFORM)
    include("${CMAKE_CURRENT_LIST_DIR}/cmake/esp.cmake")
else()
    include("${CMAKE_CURRENT_LIST_DIR}/cmake/linux.cmake")
endif()
