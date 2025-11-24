# Simulation data surfaces

## RACI: world vs. vehicle dynamics vs. I/O vs. control

| Area | Responsible | Accountable | Consulted | Informed | Notes |
| --- | --- | --- | --- | --- | --- |
| Track/config setup & reset | `World::init`, `configureTrackState`, `reset` | `World` | Mission definition, track generators | GUI/control | Loads/generates tracks, places cones, resets lap/mission counters, and seeds collision geometry.【F:sim/src/World.cpp†L173-L215】【F:sim/src/World.cpp†L387-L433】
| Mission runtime bookkeeping | `configureMissionRuntime`, `handleMissionCompletion` | `World` | Mission runtime state | Control loop | Updates mission timers/segments and enforces stop on completion.【F:sim/src/World.cpp†L434-L490】
| Vehicle dynamics | `DynamicBicycle::updateState`, wheel-speed helpers | Vehicle dynamics model | `World` for inputs, mission state for timing | GUI/logging | `World::update` pushes sanitized inputs into the bicycle model, captures wheel speeds, and accumulates distance/time.【F:sim/src/World.cpp†L217-L283】
| Collision detection | `detectCollisions`, gate/boundary segment builders | `World` | Track config | GUI/control | Detects gate crossings, lap completions, cone/boundary hits; may trigger resets or lap increments.【F:sim/src/World.cpp†L285-L386】
| Control decisions | `computeRacingControl` (path search + controller) | Control module | `World` geometry and state | GUI/logging | Generates throttle/steer based on visible cones and checkpoints; feeds lookahead indices and streams controller paths through the debug publisher.【F:sim/src/World.cpp†L138-L164】
| External control ingress | `setSvcuCommand`, public control fields | `World` | S-VCU UDP/CAN shim | Control/GUI | Stores latest S-VCU command so `World::update` can apply it when missions are running.【F:sim/src/World.cpp†L166-L215】
| Control decisions | `computeRacingControl` (path search + controller) | Control module | `World` geometry and state | GUI/logging | Generates throttle/steer based on visible cones and checkpoints; feeds lookahead indices and best-path edges for visualization.【F:sim/src/World.cpp†L138-L164】
| External control ingress | Per-frame `World::update` command input, public control fields | `World` | S-VCU UDP/CAN shim | Control/GUI | IO/control surfaces sanitize S-VCU inputs before invoking `World::update` with the neutral command for the current frame.【F:sim/src/World.cpp†L192-L274】
| Telemetry emission | `telemetry` (to `Telemetry_Update`) | Telemetry layer | `World` state | GUI/logging | Pushes authoritative physics/mission state each frame.【F:sim/src/World.cpp†L383-L386】

## Public getters consumed by GUI/control (`sim/app/fsai_run.cpp`)

* Rendering pulls cones, checkpoints, lookahead indices, and vehicle pose from the `World` getters while overlaying debug visualizations from the latest debug packet.【F:sim/app/fsai_run.cpp†L1322-L1465】
* Control loop reads and optionally overrides `World` control fields (`throttleInput`, `brakeInput`, `steeringAngle`), invokes `computeRacingControl`, and applies `setSvcuCommand` for S-VCU handoff.【F:sim/app/fsai_run.cpp†L2057-L2187】
* Rendering pulls cones, checkpoints, lookahead indices, vehicle pose, and detection overlays directly from the `World` getters for draw calls.【F:sim/app/fsai_run.cpp†L1322-L1465】
* Control loop reads and optionally overrides `World` control fields (`throttleInput`, `brakeInput`, `steeringAngle`), invokes `computeRacingControl`, and forwards sanitized inputs through `World::update` for runtime state progression.【F:sim/app/fsai_run.cpp†L2138-L2340】
* Runtime telemetry populates GUI/control panels using `World` getters for lap timing, distance, mission progress, and mission descriptors.【F:sim/app/fsai_run.cpp†L2247-L2291】

## S-VCU telemetry paths (ground truth vs. noisy sensors)

* Ground-truth values: runtime telemetry and internal control use direct state from `World`/`DynamicBicycle` (vehicle state, acceleration, mission stats) without injected noise.【F:sim/app/fsai_run.cpp†L2230-L2291】
* Noisy telemetry stream: S-VCU payloads are built from simulated measurements with Gaussian noise/latency per channel (steering, wheel RPM, torque, brake %, speed, IMU, GPS) before packaging into `Vcu2Ai*` messages; defaults fall back to noiseless ground truth when samples are missing.【F:sim/app/fsai_run.cpp†L2391-L2479】
* Ground-truth vs. request echoes: S-VCU status messages include measured signals (noisy) alongside last AI commands (e.g., brake request echoes) so consumers can compare commanded vs. sensed values.【F:sim/app/fsai_run.cpp†L2425-L2479】

## Surfaces that should be private/mediated

* Mutable `World` fields (`throttleInput`, `brakeInput`, `steeringAngle`, `useController`, `regenTrack`) are public and written by UI/control code, bypassing validation or mission state checks; debug-only data is mediated through a publisher instead of writable members.【F:sim/include/sim/World.hpp†L56-L64】【F:sim/app/fsai_run.cpp†L2057-L2187】
* Geometry accessors expose mutable vectors (cones, checkpoints) used for rendering and path planning; callers could hold stale references across resets/regenerations.【F:sim/include/sim/World.hpp†L66-L115】【F:sim/app/fsai_run.cpp†L1322-L1465】
* Telemetry uses authoritative `World` state, but S-VCU transmissions build on noisy sensor facades; separating “truth” from “telemetry” producers would clarify which consumers should see which feed.【F:sim/app/fsai_run.cpp†L2230-L2291】【F:sim/app/fsai_run.cpp†L2391-L2479】
