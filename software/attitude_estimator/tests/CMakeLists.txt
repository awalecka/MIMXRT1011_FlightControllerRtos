# Define the test executable
add_executable(unit_tests 
    test_attitude_ukf.cpp
)

# Link against your library (gnc_lib) and GoogleTest
target_link_libraries(unit_tests 
    PRIVATE 
    gnc_lib 
    gtest_main
)

# Register tests with CTest / Visual Studio Test Explorer
include(GoogleTest)
gtest_discover_tests(unit_tests)