# unit_testing/attitude_controller/CMakeLists.txt

add_flight_test(test_attitude_controller
    SOURCES 
        src/gnc/attitude_controller.cpp
        src/tests/test_attitude_controller.cpp
    INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gnc  # Adjust based on your #include style
    LIBS 
        flight_pid  # <-- Automatically injects PID source and headers!
)