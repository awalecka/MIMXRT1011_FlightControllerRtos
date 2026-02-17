# Define the static library
add_library(gnc_lib STATIC
    attitude_ukf.cpp
    attitude_ukf.hpp
)

# 1. Allow other targets (like tests) to find headers in this folder
target_include_directories(gnc_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# 2. Ensure this library can find Eigen in the root 'external' folder
target_include_directories(gnc_lib PUBLIC ${CMAKE_SOURCE_DIR}/external)