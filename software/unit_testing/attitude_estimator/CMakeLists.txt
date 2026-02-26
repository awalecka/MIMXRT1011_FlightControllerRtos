# unit_testing/attitude_estimator/CMakeLists.txt

add_flight_test(test_attitude_ukf
    SOURCES 
        src/gnc/attitude_ukf.cpp
        tests/test_attitude_ukf.cpp
    INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gnc  
)

add_flight_test(test_attitude_mekf
    SOURCES 
        src/gnc/attitude_mekf.cpp
        tests/test_attitude_mekf.cpp
    INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gnc  
)