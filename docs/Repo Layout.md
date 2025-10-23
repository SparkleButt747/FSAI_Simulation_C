

# Target repository layout (authoritative)

```
fsai/
├─ CMakeLists.txt
├─ cmake/                              # cmake helpers (toolchains, warnings, deps)
├─ .gitignore                          # ignore build/, logs/, etc.
├─ config/
│  ├─ default.yaml                     # providers, mode, rates, HUD, etc.
│  ├─ cameras/
│  │  └─ zed2i_example.yaml            # K, distortion, T_bw, fps, etc.
│  ├─ vehicle/
│  │  └─ adsdv.yaml                    # wheelbase, steer limits, MAX_SPEED, etc.
│  └─ dbc/
│     └─ adsdv.dbc                     # (or FS-AI API wrapper cfg)
├─ common/
│  ├─ include/common/
│  │  ├─ types.h                       # Frame/StereoFrame/Detections/VehicleState/ControlCmd
│  │  ├─ time/clock.h                  # monotonic ns
│  │  ├─ time/budget.h                 # perf timers
│  │  ├─ telemetry/telemetry.hpp       # structured logs
│  │  ├─ math/                          # small math utils
│  │  │  └─ geo2d.hpp
│  │  └─ spsc.h                        # lock-free SPSC
│  ├─ src/
│  │  ├─ time/clock.cpp  budget.cpp
│  │  └─ telemetry/telemetry.cpp
├─ io/
│  ├─ camera/
│  │  ├─ include/io/camera/isource.h
│  │  ├─ include/io/camera/stereo_source.h
│  │  ├─ sim_stereo/                   # minimal GL stereo renderer
│  │  │  ├─ gl_context_sdl.cpp  fbo_stereo.cpp  readback_pbo.cpp
│  │  │  ├─ cone_mesh.cpp  sim_stereo_source.cpp
│  │  └─ real_stereo/
│  │     ├─ cv/real_stereo_cv.cpp      # OpenCV/V4L2 capture
│  │     └─ zed/README.md              # (placeholder if you go native ZED SDK)
│  └─ can/
│     ├─ include/io/can/ican.h
│     ├─ providers/
│     │  ├─ fake/fake_can.cpp
│     │  └─ dbc/
│     │     ├─ dbc_gen/                # generated .c/.h from adsdv.dbc
│     │     └─ dbc_can.cpp             # SocketCAN + pack/unpack glue
│     └─ fsai_api_wrapper/             # if you adopt the FS-AI API
├─ sim/
│  ├─ include/sim/
│  │  ├─ world.hpp   # World, step(), time base
│  │  ├─ track.hpp   # PathConfig/Generator/TrackGenerator interfaces
│  │  └─ vehicle.hpp # DynamicBicycle, VehicleParam, VehicleState
│  ├─ src/
│  │  ├─ world.cpp   # orchestrates vehicle+track
│  │  ├─ track/
│  │  │  ├─ PathConfig.cpp  PathGenerator.cpp  TrackGenerator.cpp
│  │  ├─ vehicle/
│  │  │  ├─ DynamicBicycle.cpp  VehicleParam.cpp  WheelsInfo.c
│  │  └─ integration/
│  │     ├─ provider_registry.cpp  runtime_config.cpp
│  │     ├─ fake_providers.cpp     # FakeVision/FakePlanner/FakeEstimator
│  │     ├─ adapter.cpp            # Detections|truth → checkpoints for CLC
│  │     └─ path_truth.cpp         # exposes sim cones/centerline to fakes
│  └─ app/
│     └─ fsai_run.cpp              # main(), mode switch, HUD, recorder
├─ control/
│  ├─ include/control/api.h
│  ├─ src/
│  │  ├─ clc_controller.cpp        # your Custom Lookahead Controller (baseline)
│  │  ├─ control_node.cpp          # Control_Tick(): builds ControlCmd, safety, rate limits
│  │  ├─ boundary_assign.cpp       # (stub; later real)
│  │  ├─ boundary_fit.cpp          # (stub; later real)
│  │  ├─ centerline_spline.cpp     # (stub; later real)
│  │  └─ scheduler.cpp             # cadence @200 Hz, pacing to CAN (10 ms send)
│  └─ test/
│     ├─ TestSteering.c   TestThrottle.c
├─ vision/
│  ├─ include/vision/api.h
│  ├─ providers/
│  │  ├─ fake_from_sim.cpp         # PathLogic → Detections (+noise/dropout)
│  │  └─ zed_runtime.cpp           # placeholder; later ONNX/TensorRT
│  ├─ proto_py/
│  │  ├─ node.py  triangulation.py  eval/
│  └─ models/
│     ├─ cones_yolo.onnx
│     └─ model.card.yaml
├─ record/
│  ├─ include/record/api.h
│  ├─ src/recorder.cpp  replay.cpp
│  └─ logs/.gitkeep
├─ tools/
│  ├─ legacy2d/
│  │  ├─ Graphics.c  Graphics.h     # keep, but marked legacy
│  └─ scripts/
│     └─ python/plot_telemetry.py
└─ tests/
   ├─ PathLogicTest/ TestTrackGeneration.cpp
   └─ SimulationModelsTest/ TestDynamicBicycle.cpp
```


# Provider registry (simple and robust)

`sim/src/integration/provider_registry.cpp` loads providers by string (compile-time maps protected by `#ifdef`s). Read defaults from `config/default.yaml`, override via `-D` or CLI flags:

```yaml
mode: sim                # sim | hil | car
providers:
  vision: fake_from_sim  # fake_from_sim | zed
  planner: fake_track    # fake_track | spline
  estimator: fake_truth  # fake_truth | ekf
  can: fake              # fake | dbc | fsai_api
rates:
  control_hz: 200
  vision_hz: 60
  ai2vcu_period_ms: 10
```

# Interfaces (as built artifacts)

- `include/control/api.h` exposes `Control_Tick()` returning `ControlCmd`.
    
- `include/vision/api.h` exposes `Vision_ProcessStereo()` returning `Detections`.
    
- `include/io/can/ican.h` exposes `CAN_SendControl()` paced by scheduler (10 ms).
    
- `include/io/camera/stereo_source.h` exposes `grab()` for sim/real stereo.
    
- `record/include/record/api.h` exposes `Rec_*()`.
    
    
# Build hygiene & tests

- Add `-Wall -Wextra -Werror` and sanitizers in Debug at the top level.
    
- Keep your existing tests under `tests/` and `control/test/`, link them against the new libs.
    
- Make the HUD show `providers.*` and `mode` so everyone knows what’s running.
    

---