# FSAI Simulation In C

This project is a simulation-based racing application that integrates various modules such as graphics rendering, physics simulation, path logic, and telemetry to create a comprehensive testing environment.

## Key Modules

- **Graphics**: Handles rendering and graphical output. ([tools/Graphics/Graphics.c](tools/Graphics/Graphics.c), [tools/Graphics/Graphics.h](tools/Graphics/Graphics.h))
- **Simulation**: Entry point and runtime wiring for the stereo/CAN pipeline. ([sim/src/main.cpp](sim/src/main.cpp), [sim/include/sim](sim/include/sim))
- **Control**: Houses the custom lookahead controller and future pipeline stages. ([control/src](control/src), [control/include/control](control/include/control))
- **Vision**: Stereo perception stubs and future runtime implementation. ([vision/runtime_cpp](vision/runtime_cpp), [vision/proto_py](vision/proto_py))
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

Make sure you run an executable from within the build folder so that `sim/src/vehicle/Configs/configDry.yaml` can be found.

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
