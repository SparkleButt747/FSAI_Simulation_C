# Simulation data surfaces

## RACI: world vs. vehicle dynamics vs. I/O vs. control

| Area | Responsible | Accountable | Consulted | Informed | Notes |
| --- | --- | --- | --- | --- | --- |
| Track/config setup & reset | `World::init`, `configureTrackState`, `reset` | `World` | Mission definition, track generators | GUI/control | Loads/generates tracks, places cones, resets lap/mission counters, and seeds collision geometry.【F:sim/src/World.cpp†L173-L215】【F:sim/src/World.cpp†L387-L433】
| Mission runtime bookkeeping | `configureMissionRuntime`, `handleMissionCompletion` | `World` | Mission runtime state | Control loop | Updates mission timers/segments and enforces stop on completion.【F:sim/src/World.cpp†L434-L490】
| Vehicle dynamics | `DynamicBicycle::updateState`, wheel-speed helpers | Vehicle dynamics model | `World` for inputs, mission state for timing | GUI/logging | `World::update` pushes sanitized inputs into the bicycle model, captures wheel speeds, and accumulates distance/time.【F:sim/src/World.cpp†L217-L283】
| Collision detection | `detectCollisions`, gate/boundary segment builders | `World` | Track config | GUI/control | Detects gate crossings, lap completions, cone/boundary hits; may trigger resets or lap increments.【F:sim/src/World.cpp†L285-L386】
| Control decisions | `computeRacingControl` (path search + controller) | Control module | `World` geometry and state | GUI/logging | Generates throttle/steer every frame from visible cones and checkpoints; those values act as the default before AI overrides, while lookahead indices and best-path edges feed visualization.【F:sim/src/World.cpp†L138-L164】
| External control ingress | IO adapters (`WorldVehicleContext`, public control fields) | `World` | S-VCU UDP/CAN shim | Control/GUI | IO/control writes neutral throttle/brake/steer inputs each frame and replays vehicle spawns via `WorldVehicleContext` callbacks; `World::update` simply consumes those fields without capturing S-VCU commands internally.【F:sim/app/fsai_run.cpp†L1906-L1914】【F:sim/src/World.cpp†L165-L239】
| Telemetry emission | `telemetry` (to `Telemetry_Update`) | Telemetry layer | `World` state | GUI/logging | Pushes authoritative physics/mission state each frame.【F:sim/src/World.cpp†L383-L386】

## Public getters consumed by GUI/control (`sim/app/fsai_run.cpp`)

* Rendering pulls cones, checkpoints, lookahead indices, and vehicle pose directly from the `World` getters for draw calls, while debug overlays (detections, controller path) flow through the IO debug channel.【F:sim/src/world/WorldRenderAdapter.cpp†L141-L240】【F:sim/app/fsai_run.cpp†L2565-L2597】
* Control loop writes the latest sanitized control fields (`throttleInput`, `brakeInput`, `steeringAngle`, `regenTrack`), recomputes default commands via `computeRacingControl` every tick, optionally overrides them with AI inputs, and drives `VehicleDynamics::setCommand` before `World::update` consumes the resulting forces.【F:sim/app/fsai_run.cpp†L2189-L2321】
* Runtime telemetry populates GUI/control panels using `World` getters for lap timing, distance, mission progress, and mission descriptors.【F:sim/app/fsai_run.cpp†L2247-L2291】

## S-VCU telemetry paths (ground truth vs. noisy sensors)

* Ground-truth values: runtime telemetry and internal control use direct state from `World`/`DynamicBicycle` (vehicle state, acceleration, mission stats) without injected noise.【F:sim/app/fsai_run.cpp†L2230-L2291】
* Noisy telemetry stream: S-VCU payloads are built from simulated measurements with Gaussian noise/latency per channel (steering, wheel RPM, torque, brake %, speed, IMU, GPS) before packaging into `Vcu2Ai*` messages; defaults fall back to noiseless ground truth when samples are missing.【F:sim/app/fsai_run.cpp†L2391-L2479】
* Ground-truth vs. request echoes: S-VCU status messages include measured signals (noisy) alongside last AI commands (e.g., brake request echoes) so consumers can compare commanded vs. sensed values.【F:sim/app/fsai_run.cpp†L2425-L2479】

## Surfaces that should be private/mediated

* Mutable `World` fields (`throttleInput`, `brakeInput`, `steeringAngle`, `regenTrack`) are public and written by UI/control code each frame, so writing straight to them bypasses further validation or mission state checks.【F:sim/include/sim/World.hpp†L50-L56】【F:sim/app/fsai_run.cpp†L2179-L2325】
* Geometry accessors expose mutable vectors (cones, checkpoints) used for rendering and path planning; callers could hold stale references across resets/regenerations.【F:sim/include/sim/World.hpp†L60-L114】【F:sim/app/gui_world_adapter.cpp†L5-L18】
* Telemetry uses authoritative `World` state, but S-VCU transmissions build on noisy sensor facades; separating “truth” from “telemetry” producers would clarify which consumers should see which feed.【F:sim/app/fsai_run.cpp†L2230-L2291】【F:sim/app/fsai_run.cpp†L2391-L2479】
