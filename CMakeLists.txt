cmake_minimum_required(VERSION 3.16)
project(SatelliteOrbitSim VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Compiler warnings
add_compile_options(-Wall -Wextra -Wpedantic -O2)

# Main library (all simulation code)
add_library(orbit_lib
    src/Vector3D.cpp
    src/CelestialBody.cpp
    src/Satellite.cpp
    src/GravityModel.cpp
    src/Integrator.cpp
    src/Simulation.cpp
    src/OrbitalElements.cpp
    src/CSVExporter.cpp
)
target_include_directories(orbit_lib PUBLIC ${CMAKE_SOURCE_DIR}/include)

# Main executable
add_executable(orbit_sim src/main.cpp)
target_link_libraries(orbit_sim PRIVATE orbit_lib)

# Multi-threaded executable
add_executable(orbit_sim_mt src/main_multithreaded.cpp)
target_link_libraries(orbit_sim_mt PRIVATE orbit_lib)
find_package(Threads REQUIRED)
target_link_libraries(orbit_sim_mt PRIVATE Threads::Threads)

# Tests
add_executable(run_tests tests/test_main.cpp)
target_link_libraries(run_tests PRIVATE orbit_lib)
