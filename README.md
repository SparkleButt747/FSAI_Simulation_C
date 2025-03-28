# FSAI Simulation In C

A brief description of your project goes here. For example:

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
3. Run the following commands:
4. (Apple Silicon macs might and will have to change the include paths in the CMakeLists.txt, this will cause all sorts of merge conflicts, so make sure to clone once and have a .gitignore setup or something)

   ```sh
   cd build
   cmake ..
   make

