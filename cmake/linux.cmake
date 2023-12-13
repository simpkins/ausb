set(CMAKE_CXX_STANDARD 20)

include(CTest)

# Enable ASAN in debug builds if we are using clang
if(CMAKE_CXX_COMPILER_ID STREQUAL Clang)
    string(APPEND CMAKE_CXX_FLAGS_DEBUG " -fsanitize=address")
endif()
if(CMAKE_C_COMPILER_ID STREQUAL Clang)
    string(APPEND CMAKE_C_FLAGS_DEBUG " -fsanitize=address")
endif()

string(APPEND CMAKE_CXX_FLAGS_DEBUG " -Werror")
string(APPEND CMAKE_C_FLAGS_DEBUG " -Werror")

add_library(ausb STATIC ${srcs})
target_include_directories(
    ausb PUBLIC
    "${AUSB_ROOT_DIR}"
)

add_executable(test_runner test/runner/mock/main.cpp)
target_link_libraries(test_runner PUBLIC test_cases)

add_test(NAME ausb COMMAND ausb_test)
