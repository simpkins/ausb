# Enable ASAN in debug builds if we are using clang
if(CMAKE_CXX_COMPILER_ID STREQUAL Clang)
    string(APPEND CMAKE_CXX_FLAGS_DEBUG " -fsanitize=address")
endif()
if(CMAKE_C_COMPILER_ID STREQUAL Clang)
    string(APPEND CMAKE_C_FLAGS_DEBUG " -fsanitize=address")
endif()

string(APPEND CMAKE_CXX_FLAGS_DEBUG " -Werror")
string(APPEND CMAKE_C_FLAGS_DEBUG " -Werror")
