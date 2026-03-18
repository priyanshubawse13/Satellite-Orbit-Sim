# Satellite Orbit Simulation & Analysis Tool

A cross-platform C++17 application that simulates satellite orbital mechanics using numerical integration methods. Models gravitational forces including Earth's J2 oblateness perturbation, atmospheric drag, and supports multiple integration schemes (Euler, RK4, RKF45).

## Features

- **Orbital Mechanics**: Keplerian orbital element computation, state vector ↔ elements conversion
- **Gravity Models**: Point-mass gravity, J2 oblateness perturbation, exponential atmospheric drag
- **Numerical Integration**: Euler (1st order), Classical RK4, and adaptive RKF45 methods
- **Multi-threaded**: Parallel simulation of satellite constellations using C++17 `std::thread`
- **Data Export**: CSV output of trajectories, orbital elements, and simulation summaries
- **Validation**: Energy conservation tests, Euler vs RK4 accuracy comparison, orbital element roundtrip

## Architecture
SatelliteOrbitSim/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── Vector3D.h          # 3D vector math (operators, dot, cross, normalize)
│   ├── CelestialBody.h     # Planetary bodies (Earth, Moon, Mars factories)
│   ├── Satellite.h          # Satellite state (position, velocity, time)
│   ├── GravityModel.h      # Force models (point-mass, J2, drag)
│   ├── Integrator.h        # Numerical methods (Euler, RK4, RKF45)
│   ├── OrbitalElements.h   # Keplerian elements ↔ state vector conversion
│   ├── Simulation.h        # Simulation engine (single & batch)
│   └── CSVExporter.h       # Data export to CSV
├── src/
│   ├── *.cpp               # Implementation files
│   ├── main.cpp            # Single-threaded simulation (3 satellites)
│   └── main_multithreaded.cpp  # Multi-threaded constellation (8 satellites)
├── tests/
│   └── test_main.cpp       # Unit tests
└── data/                    # CSV output directory
## Build & Run
```bash
# Build
mkdir build && cd build
cmake .. && make -j$(sysctl -n hw.ncpu)

# Run tests
./run_tests

# Single-threaded simulation
./orbit_sim

# Multi-threaded simulation
./orbit_sim_mt
```

## Tech Stack

- **C++17**: `std::thread`, `std::mutex`, `std::shared_ptr`, structured bindings
- **CMake 3.16+**: Cross-platform build system
- **STL**: Extensive use of containers, algorithms, and functional utilities
- **GDB-compatible**: Debug builds supported via `cmake -DCMAKE_BUILD_TYPE=Debug ..`

## Simulated Scenarios

| Satellite | Orbit Type | Altitude | Period |
|-----------|-----------|----------|--------|
| ISS | Circular LEO | 408 km | ~92 min |
| Molniya | Highly Elliptical | 500–39,873 km | ~12 hr |
| GEO | Geostationary | 35,786 km | ~24 hr |

## Author

Priyanshu Bawse — M.S. Computer Engineering, Iowa State University
