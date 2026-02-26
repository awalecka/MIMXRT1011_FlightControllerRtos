cmake_minimum_required(VERSION 3.25)
project(PIDProject)

# Use C++23 as requested
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(GOOGLETEST_VERSION 1.14.0)

# --- Dependencies ---
# Prevent GoogleTest from overriding parent compiler settings (optional but recommended)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Point to the local folder instead of fetching from URL
add_subdirectory(external/googletest)

# --- Library Target ---
# We define the library sources relative to the root
add_library(pid_lib
    source/pid/pid_controller.cpp
    source/pid/pid_controller.h
)

# Allow the library and tests to find the headers
target_include_directories(pid_lib PUBLIC source/pid)

# --- Test Target ---
enable_testing()

add_executable(pid_tests
    source/tests/test_pid_controller.cpp
)

# Link against the library and the local GoogleTest main runner
target_link_libraries(pid_tests 
    PRIVATE 
    pid_lib 
    gtest_main
)

# Register tests with CTest
include(GoogleTest)
gtest_discover_tests(pid_tests)