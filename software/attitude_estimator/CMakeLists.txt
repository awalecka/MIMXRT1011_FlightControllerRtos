cmake_minimum_required(VERSION 3.25)
project(FlightControllerRef)

# Standard C++23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler Options & Fixes
if(MSVC)
    # Enable standard warnings, but turn off "Warnings as Errors" (/WX) 
    # to prevent external library warnings (like Eigen) from stopping the build.
    add_compile_options(/W4)

    # FIX: Switch from /Zi (Separate PDB) to /Z7 (Embedded Debug Info).
    # This prevents the "limit exceeded for program database" link error caused by 
    # Eigen's heavy template usage.
    foreach(lang C CXX)
        string(REPLACE "/Zi" "/Z7" CMAKE_${lang}_FLAGS_DEBUG "${CMAKE_${lang}_FLAGS_DEBUG}")
        string(REPLACE "/Zi" "/Z7" CMAKE_${lang}_FLAGS_RELWITHDEBINFO "${CMAKE_${lang}_FLAGS_RELWITHDEBINFO}")
    endforeach()
endif()

# Global include path for libraries in 'external' folder (e.g. Eigen)
include_directories(external)

# 1. Build the GNC Library
add_subdirectory(src/gnc)

# 2. Build Testing Infrastructure
option(BUILD_TESTING "Build Unit Tests" ON)
if(BUILD_TESTING)
    enable_testing()
    
    # GoogleTest Configuration
    set(GOOGLETEST_VERSION "1.14.0") 
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    # Add GoogleTest project
    add_subdirectory(external/googletest)
    
    # Add your test folder
    add_subdirectory(tests)
endif()