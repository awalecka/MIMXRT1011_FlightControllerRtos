# unit_testing/pid/CMakeLists.txt

add_flight_test(test_pid_controller
    SOURCES 
        src/pid/pid_controller.cpp
        src/tests/test_pid_controller.cpp
    INCLUDES
        # Point this to wherever the root of your "controllers/pid_controller.h" lives.
        # Assuming the file includes it like `#include "controllers/pid_controller.h"`,
        # the include path should be the directory containing the 'controllers' folder.
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pid 
)