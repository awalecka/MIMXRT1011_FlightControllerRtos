# unit_testing/pid/CMakeLists.txt

# 1. Compile the PID logic into a reusable static library
add_library(flight_pid STATIC src/pid/pid_controller.cpp)

# 2. Expose the EXACT folder containing pid_controller.h to anyone who links this library
target_include_directories(flight_pid PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/pid) # <-- Added /pid here

# 3. Create the standalone PID unit test
add_flight_test(test_pid_controller
    SOURCES 
        src/tests/test_pid_controller.cpp
    LIBS 
        flight_pid 
)