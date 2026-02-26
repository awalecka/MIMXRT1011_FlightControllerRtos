# unit_testing/nmea/CMakeLists.txt

add_flight_test(test_nmea_parser
    SOURCES 
        src/sensors/nmea.cpp
        src/tests/test_nmea_parser.cpp
    INCLUDES
        # Point this to the directory that allows your test file to find the header.
        # If test_nmea_parser.cpp uses `#include "nmea.h"`, use:
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sensors
)