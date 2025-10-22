# FSAI Simulation In C

This project is a simulation-based racing application that integrates various modules such as graphics rendering, physics simulation, path logic, and telemetry to create a comprehensive testing environment.

## Key Modules

- **Graphics**: Handles rendering and graphical output. ([Graphics/Graphics.c](Graphics/Graphics.c), [Graphics/Graphics.h](Graphics/Graphics.h))
- **MainController**: Contains the main logic for controlling cars and other entities. ([MainController/CarController.c](MainController/CarController.c))
- **PathLogic**: Includes assets and logic for pathfinding and navigation.
- **PhysicsEngine**: Simulates the physics of the racing environment.
- **RacingAlgorithms**: Implements racing strategies and algorithms.
- **Telemetry**: Manages telemetry data and logging.

## Build Instructions

To build the project, follow these steps:

1. Ensure you have CMake installed on your system.
2. Navigate to the root directory of the project.
3. Run the following commands or use the `build.sh`/`rebuild.sh` scripts:

   ```sh
   cd build
   cmake ..
   make

   ```

4. The CMakeLists.txt should now automatically detect architecture (only arm and not arm) and use the correct paths

## Running the executables

Make sure you run an executable from within the build folder so that `configDry.yaml` can be found.

## Building BoundaryEstimation.cpp

1. Make sure you have installed `cgal` and `qt6`.
2. Create a directory at planning/build
3. Run:

```sh
  cd build
  cmake ..
  make
```

or use the `build.sh`/`rebuild.sh` scripts.

Note on inlcuding headers, just use the file name and ensure there are no duplicate file names, cmake will grab the correct file...
