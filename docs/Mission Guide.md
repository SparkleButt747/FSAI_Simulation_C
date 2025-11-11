# Mission guide

This document explains how the simulator chooses and runs missions, and how to
extend the catalogue.

## Mission selection UX

`fsai_run` exposes four built-in missions (Acceleration, Skidpad, Autocross, and
Trackdrive). When the process starts without the `--mission` flag and standard
input is interactive, the executable prints a numbered menu and waits for user
input. Enter a number from the menu or one of the supported tokens (for example
`accel`, `skid`, `auto`, or `track`) to pick a mission; pressing enter selects
Autocross, the default choice. If stdin is not interactive (for example when the
simulator is launched from an IDE without a terminal) the same Autocross default
is used automatically.【F:sim/app/fsai_run.cpp†L62-L189】

Pass `--mission <value>` to bypass the prompt. The parser accepts the same
numbers and textual tokens, and rejects missing or invalid arguments with a
logged error before the process exits.【F:sim/app/fsai_run.cpp†L1236-L1263】

## Mission behaviour reference

The following table summarises the lap segmentation and stop behaviour driven by
`MissionRuntimeState` and `World`.

| Mission      | Track source                      | Lap pattern                           | Stop behaviour | Notes |
|--------------|-----------------------------------|----------------------------------------|----------------|-------|
| Acceleration | `configs/tracks/acceleration.csv` | One timed lap                          | Brake on finish| Straight-line drag strip. |【F:sim/app/fsai_run.cpp†L200-L206】【F:sim/src/MissionRuntimeState.cpp†L18-L70】【F:sim/src/World.cpp†L320-L336】
| Skidpad      | `configs/tracks/skidpad.csv`      | Warmup + 2 timed laps + exit lap       | Brake on finish| Simplified single loop; no separate CW/CCW handling yet. |【F:sim/app/fsai_run.cpp†L206-L214】【F:sim/src/MissionRuntimeState.cpp†L71-L116】【F:configs/tracks/skidpad.csv†L1-L40】【F:sim/src/World.cpp†L320-L336】
| Autocross    | Procedurally generated            | One timed lap                          | Brake on finish| Track regenerates on reset if allowed. |【F:sim/app/fsai_run.cpp†L214-L222】【F:sim/src/MissionRuntimeState.cpp†L71-L116】【F:sim/src/World.cpp†L338-L376】
| Trackdrive   | Procedurally generated            | Ten timed laps                         | Brake on finish| Shares regeneration logic with Autocross. |【F:sim/app/fsai_run.cpp†L218-L222】【F:sim/src/MissionRuntimeState.cpp†L71-L116】【F:sim/src/World.cpp†L338-L376】

`World::handleMissionCompletion()` applies zero throttle and full brake for all
missions once the configured lap segments finish, ensuring a predictable stop in
the virtual environment.【F:sim/src/World.cpp†L320-L336】

The CSV schema for the built-in tracks, including the column definitions and tag
usage, lives in `configs/tracks/README.md`. Use the same format if you author
new layouts for acceleration-style missions.【F:configs/tracks/README.md†L1-L32】

## Extending the mission list

To add a new mission:

1. Define a `MissionOption` entry in `kMissionOptions` with the descriptor and
   any useful selection tokens.【F:sim/app/fsai_run.cpp†L62-L124】
2. Extend `BuildMissionDefinition()` with the new descriptor case to configure
   the track source, lap target, and regeneration behaviour. Point CSV-based
   missions at assets under `configs/tracks/`.【F:sim/app/fsai_run.cpp†L200-L222】
3. Update `MissionRuntimeState::ConfigureSegments()` if the mission needs a
   unique warmup/timed/exit sequence beyond the existing templates.【F:sim/src/MissionRuntimeState.cpp†L71-L116】
4. If the mission relies on special stop handling or progress tracking, update
   `World::configureMissionRuntime()` or related helpers accordingly.【F:sim/src/World.cpp†L300-L376】

Following this flow keeps the CLI prompt, runtime telemetry, and mission control
logic consistent.
