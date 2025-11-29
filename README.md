# FSAI Simulation In C

This project is a simulation-based racing application that integrates various modules such as graphics rendering, physics simulation, path logic, and telemetry to create a comprehensive testing environment.

## Key Modules

- **Graphics**: Handles rendering and graphical output. ([Graphics/Graphics.c](Graphics/Graphics.c), [Graphics/Graphics.h](Graphics/Graphics.h))
- **MainController**: Contains the main logic for controlling cars and other entities. ([MainController/CarController.c](MainController/CarController.c))
- **PathLogic**: Includes assets and logic for pathfinding and navigation.
- **PhysicsEngine**: Simulates the physics of the racing environment.
- **Telemetry**: Manages telemetry data and logging.

## Build Instructions

### Dependencies

Install the build toolchain and runtime libraries before configuring the project:

- **Common:** CMake (3.20+), a C/C++ compiler, `pkg-config`.
- **Math/geometry:** Eigen3, CGAL (with the Qt6 viewer component).
- **Graphics:** SDL2, OpenGL development headers.
- **Computer Vision:** ONNXRuntime, OpenCV
- **Config/data:** yaml.

Ubuntu/Debian example:

```sh
sudo apt-get update
sudo apt-get install build-essential cmake pkg-config libsdl2-dev libyaml-dev \
     libeigen3-dev libopengl-dev libcgal-dev libcgal-qt6-dev qt6-base-dev
```

macOS (Homebrew) example:

```sh
brew install cmake sdl2 yaml-cpp eigen cgal
```

To install ONNXRuntime and OpenCV, follow instructions online.

### Configure & build

1. Navigate to the root directory of the project.
2. Generate the build tree with CMake (or use the helper scripts):

   ```sh
   mkdir -p build
   cd build
   cmake ..
   make -j
   ```

   The provided `build.sh` and `rebuild.sh` scripts wrap the same steps.

3. `CMakeLists.txt` will automatically detect the host architecture (ARM vs. x86) and pick the correct library paths.

## Runtime GUI and controls

Launch the simulator GUI from the build directory so that assets such as `configDry.yaml` resolve correctly:

```sh
./fsai_run
```

The executable opens an SDL window with Dear ImGui panels:

- **Simulation Stats** – vehicle pose, kinematics, axle torques, brake system, wheel RPM, accelerations, and lap timing. Click the collapsing headers to expand/collapse individual sections.
- **Control Panel** – shows manual inputs, AI requests, applied commands, and CAN acknowledgement timers. Color highlights indicate data freshness.
- **CAN Telemetry** – streaming feedback from the simulated VCU. Each section is collapsible; stale data is colored amber/red.
- **Simulation Log** – the scrolling log console that previously streamed to stdout. Use the `Clear` button to flush messages, toggle `Auto-scroll` to stay at the newest entry, and scroll with the mouse wheel to review history.

Keyboard and mouse interactions:

- Press **`M`** to toggle between automatic (controller) and manual driving modes.
- In manual mode use **`W`** (throttle), **`S`** (brake), and **`A` / `D`** (steering). Keep the terminal window focused for these inputs because they are read from standard input.
- Press **`Q`** to quit the simulator.
- Click and drag inside any panel to scroll; right-click a panel title bar to access the standard ImGui window options.

When running with the S-VCU stub, the helper script wires everything together:

```sh
tools/scripts/run_sim_with_svcu.sh
```

The script prints the process IDs and then launches the GUI-enabled simulator.

## Mission selection and configuration

When `fsai_run` starts without a mission override it checks whether standard
input is interactive. If it can prompt the user, a numbered mission menu is
printed; otherwise the simulator defaults to **Autocross** (option 3 in the
list).【F:sim/app/fsai_run.cpp†L148-L171】 Enter either the displayed number or
one of the mission tokens (for example `accel`, `skid`, `auto`, or `track`) to
choose a mission.【F:sim/app/fsai_run.cpp†L62-L124】 Pressing enter without
typing anything accepts the highlighted default.【F:sim/app/fsai_run.cpp†L172-L189】

To bypass the prompt pass `--mission <value>` on the command line. The flag
accepts the same numbers and tokens that the interactive prompt lists; invalid
values cause the simulator to exit with an error message.【F:sim/app/fsai_run.cpp†L1236-L1263】

Acceleration and skidpad missions load their cone layouts from
`configs/tracks/acceleration.csv` and `configs/tracks/skidpad.csv`
respectively.【F:sim/app/fsai_run.cpp†L200-L217】 The CSV schema and guidelines
for creating custom layouts live in `configs/tracks/README.md`.

## Mission lifecycle overview

Mission timing and stopping behaviour are coordinated by
`MissionRuntimeState` and `World`:

- **Acceleration** – one timed lap on the straight-line CSV track. Mission
  time advances until the checkpoint at the far end is crossed, then the
  simulator commands a full brake application to stop the car.【F:sim/app/fsai_run.cpp†L200-L206】【F:sim/src/MissionRuntimeState.cpp†L18-L70】【F:sim/src/World.cpp†L320-L336】
- **Skidpad** – four laps total: one warmup, two timed laps, and one exit lap.
  The layout is a simplified single-loop CSV that does not yet model separate
  clockwise/counter-clockwise circles. After the exit lap the stop command is
  issued in the same way as the other missions.【F:sim/app/fsai_run.cpp†L206-L214】【F:sim/src/MissionRuntimeState.cpp†L71-L116】【F:configs/tracks/skidpad.csv†L1-L40】【F:sim/src/World.cpp†L320-L336】
- **Autocross** – one timed lap on a procedurally generated random track. If
  cone collisions occur and the mission allows regeneration, resetting the
  simulation produces a fresh layout.【F:sim/app/fsai_run.cpp†L214-L222】【F:sim/src/MissionRuntimeState.cpp†L71-L116】【F:sim/src/World.cpp†L338-L376】
- **Trackdrive** – ten timed laps on a random track with the same regeneration
  logic as Autocross.【F:sim/app/fsai_run.cpp†L218-L222】【F:sim/src/MissionRuntimeState.cpp†L71-L116】【F:sim/src/World.cpp†L338-L376】

`World::handleMissionCompletion()` zeros the throttle, applies full brake, and
marks that the stop command was sent so the simulator does not repeatedly log
the message.【F:sim/src/World.cpp†L320-L336】 This behaviour applies to every
mission once the configured lap segments are complete.【F:sim/src/MissionRuntimeState.cpp†L41-L70】

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
