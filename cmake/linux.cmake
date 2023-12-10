set(CMAKE_CXX_STANDARD 20)

include(CTest)

# Prefer compiling clang if we can find it.
# Enable ASAN if we found clang.
find_program(CXX_COMPILER NAMES "clang++" "clang++-15" "g++")
find_program(C_COMPILER NAMES "clang" "clang-15" "gcc")
if(CXX_COMPILER)
    set(CMAKE_CXX_COMPILER "${CXX_COMPILER}")
    string(REGEX MATCH ".*/clang.*" IS_CLANG "${CXX_COMPILER}")
    if(IS_CLANG)
      string(APPEND CMAKE_CXX_FLAGS_DEBUG " -fsanitize=address")
    endif()
endif()
if(C_COMPILER)
    set(CMAKE_C_COMPILER "${C_COMPILER}")
    string(REGEX MATCH ".*/clang.*" IS_CLANG "${C_COMPILER}")
    if(IS_CLANG)
      string(APPEND CMAKE_C_FLAGS_DEBUG " -fsanitize=address")
    endif()
endif()

string(APPEND CMAKE_CXX_FLAGS_DEBUG " -Werror")
string(APPEND CMAKE_C_FLAGS_DEBUG " -Werror")

add_library(ausb STATIC ${srcs})
target_include_directories(
    ausb PUBLIC
    "${AUSB_ROOT_DIR}"
)

add_executable(ausb_test test/runner/mock/main.cpp)
add_test(NAME ausb COMMAND ausb_test)
