# unit_testing/nav_estimator/CMakeLists.txt

set(NAV_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/gnc)
set(NMEA_SRC_DIR ${CMAKE_SOURCE_DIR}/nmea/src/sensors)

# Test core math and algorithm components: LTP, MEKF, StationaryDetector
add_flight_test(test_nav_core
    SOURCES
        src/gnc/local_tangent_plane.cpp
        src/gnc/ins_mekf.cpp
        src/gnc/stationary_detector.cpp
        tests/test_local_tangent_plane.cpp
        tests/test_ins_mekf.cpp
        tests/test_stationary_detector.cpp
    INCLUDES
        ${NAV_SRC_DIR}
)

# Test top-level navigator scenarios and vehicle simulation
add_flight_test(test_nav_scenarios
    SOURCES
        src/gnc/local_tangent_plane.cpp
        src/gnc/ins_mekf.cpp
        src/gnc/stationary_detector.cpp
        src/gnc/ins_navigator.cpp
        ${NMEA_SRC_DIR}/nmea.cpp
        tests/sim/vehicle_sim.cpp
        tests/test_ins_navigator.cpp
    INCLUDES
        ${NAV_SRC_DIR}
        ${NMEA_SRC_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/sim
)
