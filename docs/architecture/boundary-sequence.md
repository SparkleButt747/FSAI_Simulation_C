# Command and telemetry boundaries

The new boundary interfaces (`world::IWorldView`, `vehicle::IVehicleDynamics`, `io::IIoFacade`) clarify how control, dynamics, and rendering exchange data without sharing mutable state. IO is the only conduit that both control and perception touch; the world exposes ground truth solely to IO (for telemetry/stereo frames) and to vehicle dynamics (for stepping physics).

## Command/telemetry loop (no noise)

```mermaid
sequenceDiagram
    participant Control as Control loop / UI
    participant IO as io::IIoFacade
    participant World as world::IWorldView
    participant Dyn as vehicle::IVehicleDynamics

    Control->>IO: push_command(FsaiControlCmd)
    IO->>World: publish_ground_truth(world)
    World->>Dyn: set_command(throttle, brake, steer)
    Dyn-->>World: state(), transform(), wheels_info()
    World-->>IO: vehicle_state(), track geometry
    World-->>IO: publish_debug_packet(WorldDebugPacket)
    IO-->>Control: latest_raw_telemetry()
```

## Command/telemetry loop (noise-enabled)

```mermaid
sequenceDiagram
    participant Control as Control loop / UI
    participant IO as io::IIoFacade
    participant World as world::IWorldView
    participant Dyn as vehicle::IVehicleDynamics

    Control->>IO: push_command(ControlCommand)
    IO->>World: publish_ground_truth(world)
    World->>Dyn: set_command(throttle, brake, steer)
    Dyn-->>World: state(), transform(), wheels_info()
    World-->>IO: vehicle_state(), track geometry
    World-->>IO: publish_debug_packet(WorldDebugPacket)
    IO->>IO: set_noise_config(TelemetryNoiseConfig)
    IO-->>Control: latest_noisy_telemetry(), latest_stereo_frame()
```

These sequences highlight the dependency direction: control and perception communicate only through IO, while the world remains the source of truth for both physics and rendered sensor products.
