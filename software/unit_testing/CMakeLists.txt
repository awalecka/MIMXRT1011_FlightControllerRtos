cmake_minimum_required(VERSION 3.14)
project(FlightControllerUnitTests CXX)

# Enforce C++23 globally for the tests as requested previously
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------------------------------------------------------
# 1. External Dependencies
# ------------------------------------------------------------------------------

# Automatically fetch GoogleTest. This prevents checking massive GTest 
# source trees into your version control.
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.14.0 # Or whichever version you prefer
)
# For Windows MSVC: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Setup a shared Eigen interface library. 
# Any target that links to 'eigen' will automatically get this include path.
add_library(eigen INTERFACE)
target_include_directories(eigen INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/external/Eigen)

# ------------------------------------------------------------------------------
# 2. Test Generator Helper Macro
# ------------------------------------------------------------------------------
# This function abstracts away all the boilerplate required to create a new test.
function(add_flight_test TARGET_NAME)
    # Parse named arguments: SOURCES, INCLUDES, and LIBS
    cmake_parse_arguments(ARG "" "" "SOURCES;INCLUDES;LIBS" ${ARGN})

    add_executable(${TARGET_NAME} ${ARG_SOURCES})

    # Automatically link GTest, GMock, and Eigen to every test we create
    target_link_libraries(${TARGET_NAME} PRIVATE 
        GTest::gtest_main 
        GTest::gmock
        eigen
        ${ARG_LIBS}
    )

    # Apply specific include directories for this module
    target_include_directories(${TARGET_NAME} PRIVATE ${ARG_INCLUDES})

    # Automatically discover and register tests with CTest
    include(GoogleTest)
    gtest_discover_tests(${TARGET_NAME})
endfunction()

# ------------------------------------------------------------------------------
# 3. Add Sub-Modules
# ------------------------------------------------------------------------------
# Enable CTest runner
enable_testing()

add_subdirectory(pid)                  # Added first so 'flight_pid' is available
add_subdirectory(attitude_controller)  # Can now safely link against 'flight_pid'
add_subdirectory(nmea)
add_subdirectory(attitude_estimator)
add_subdirectory(rls_mag_calibration)