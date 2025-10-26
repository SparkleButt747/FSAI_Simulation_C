# FS-AI Pipelines & Contracts v1.1 — Stereo + Cone Output

_Date:_ 26 Oct 2025
_Scope:_ Incorporates team requirements: **Vision requires Left/Right frames** (real or simulated). **Control consumes cone positions**. **Simulation** must produce synchronized stereo imagery and switch to real cameras/CAN for on‑car tests.

---

## Delta from v1.0 (what changed)

- **Vision I/O:** Input is **stereo** (Left/Right) frames. Output is **cone detections** (3D positions + side + confidence). Track construction moves to **Control**.

- **Simulation:** 2D top‑down isn’t sufficient. Add a minimal **3D renderer** that outputs synchronized stereo images matching real camera intrinsics/extrinsics.

- **Swap to real‑world:** Replace `SimStereoCamera` → `RealStereoCamera`, and `SimVehicleModel` → `CAN/Hypermotive` with identical data contracts.

- **Control:** Boundary assign removed. It only makes up a part of the cost function and the correct path through the cones can still be estimated without any color information. Backend of the pipeline was also simplified to use the existing racing algorithm instead of using a speed profile and LQR, Stanley etc. controllers for lateral and longitudinal.

---

## Team Pipelines

### 1) Vision Team — Stereo → Cones

**Goal:** Convert synchronized Left/Right frames into cone positions in the vehicle body frame ((\mathcal{B})).

**Input**

- `StereoFrame`: two `Frame`s: `left`, `right`. Each frame includes `t_ns`, `K` (intrinsics), distortion model, and extrinsics `T_bw` (body←camera). Left/Right are **time‑synchronized**.

**Output**

- `Detections` containing up to `N_MAX` `ConeDet` entries with:

  - `p_B = (x,y,z)` in meters in body frame.

  - `side ∈ {LEFT, RIGHT, UNKNOWN}`

  - `conf ∈ [0,1]`

  - Optional `cov` (upper‑triangular 3×3, 6 floats) for downstream fusion.

**Runtime Pipeline**

```
StereoFrame -> Rectify/Undistort -> Inference (per‑eye or fused) -> Matching/Triangulation ->
Side Classification -> Outlier Rejection (RANSAC/consistency) -> Emit ConeDet[] in body frame
```

- **Rectify/Undistort:** Use `K` and distortion params. Enforce identical resolutions.

- **Inference:** Small detector (classifies cone + color/side if available). OK to start with HSV heuristics in sim; ship ONNX for prod.

- **Triangulation:** Feature correspondences (epipolar constraints) or center-of-mass per detection.

- **Side class:** From color in sim/real if reliable; otherwise learned classifier. If unknown, set `UNKNOWN` and let Control route conservatively.

- **Latency budget:** (\le 20) ms E2E @ 1280×720 on target.

**Python → C++ path (no surprises):**

- Prototype in Python using shared memory for `StereoFrame` and write `Detections` to an SPSC queue. Keep the **exact C structs**.

- Replace Python inference with ONNX Runtime C++ once metrics/budget pass. No API changes.

**Acceptance (ship gates):**

- mAP@0.5 on cone class ≥ 0.90 (sim), ≥ 0.80 (real dataset subset).

- Depth RMSE ≤ 0.25 m up to 30 m.

- E2E latency ≤ 20 ms, drop rate < 1%.

---

### 2) Control Team — Cones → Track → Commands

**Goal:** Build a navigable track from cones and output steering/throttle.

**Input**

- `Detections{t_ns, ConeDet[]}` (Vision)

- `VehicleState` (Sim truth for now; EKF on car)

**Output**

- `ControlCmd{steer_rad, throttle, brake, t_ns}` → CAN messages via a CAN bus to the (simulated) VCU

**Pipeline**

```
Cones -> Centerline (Boundary Estimation) -> Racing Algorithm -> Command Scheduler (rate limits, interlocks) -> Output
```

- **Centerline:** midpoint of cones, If using splines, enforce C¹/C² continuity. Horizon 60–80 m.

- **Controllers:** steering @ 200 Hz; speed @ 200 Hz. Rate‑limit steering and throttle/brake.

- **Latency handling:** forward‑predict `VehicleState` to `t_now + τ_{vision}`.

**Acceptance:** closed‑loop SIL converges to lap completion with no boundary violations at target μ; HIL meets CAN timing budgets.

---

### 3) Simulation/Integration — Stereo Renderer + Switching

**Goal:** Provide synchronized stereo frames in sim; record/replay; switch to real cameras and CAN for on‑car.

**Input/Output**

- Consumes `ControlCmd` from Control; applies to Dynamic Bicycle model in sim.

- Produces `StereoFrame` for Vision; handles record/replay for all channels.

**Pipeline (Sim mode)**

```
Clock(fixed) -> Vehicle/World Integrator (200 Hz) ->
Renderer (3D) -> Offscreen FBOs (Left/Right) -> Readback -> StereoFrame -> Vision -> Control ->
Apply Control to model -> HUD/Telemetry -> Record
```

**Pipeline (Real mode)**

```
RealStereoCamera -> StereoFrame -> Vision -> Control -> CAN/Hypermotive
```

**Stereo Synchronization:**

- Time skew |t_left − t_right| ≤ **1 ms**. If not hardware‑synced, Vision must reject.

- `T_bw` must be accurate for both eyes; left/right extrinsics relation (`T_br_bl`) stored in config.

**3D Renderer Requirements (minimal)**

- Two pinhole cameras with configurable `K` and baseline `B`.

- Cones as 3D meshes or billboards at correct heights; simple ground plane.

- Effects (optional, for realism): motion blur toggle, exposure jitter, noise, lens distortion.

- Offscreen rendering to two textures at the exact real resolution; copy to CPU buffer for `Frame.data`.

- Implementation options:

  - SDL2 window + OpenGL 3.3 Core (fast path, lowest friction), or

**Record/Replay**

- Record channels: `stereo_frame`, `detections`, `state_gt`, `state_est`, `cmd`, `can_rx`, `can_tx`.

- Replay feeds Vision/Control for deterministic SIL.

---

## Cross‑Team Requirements (Non‑Negotiable)

1. **One set of structs (ABI):** `common/types.h` governs. Breaking changes bump `FSAI_ABI_MAJOR` and require coordinated PRs.

2. **Time discipline:** All messages carry `t_ns` from the same monotonic clock. Vision outputs are stamped to sensor time (mid‑exposure if known).

3. **No heap on hot paths:** Vision postproc, Planner, Controller, and Sim tick use fixed buffers/arenas.

4. **Budgets:** Vision ≤ 20 ms; Planner+Controller ≤ 2 ms; Render ≤ 6 ms (60 Hz); CAN send ≤ 5 ms from `ControlCmd.t_ns`; dropped frames < 1%.

5. **Calibration is an artifact:** Intrinsics/extrinsics/distortion live in YAML with checksum and version; logs include calibration version.

6. **DBC‑only CAN:** All packing/unpacking generated from DBC.

7. **Record/Replay first:** Every feature lands with a replay test.

8. **CI gates:** clang‑tidy, `-Wall -Wextra -Werror`, sanitizers in debug, perf tests on CI hardware; model hash validated before load.

9. **Threading discipline:** SPSC queues across modules; atomic source switching; no global locks.

10. **Safety/Degradation:** Stale inputs (>50 ms) → freeze planner and decay to zero within 200 ms; invalid detections → slow down.

---

## Data Contracts (put in `common/types.h`)

```c
// Pixel formats kept minimal and identical across sim/real paths
typedef enum { PIXEL_RGB888=0, PIXEL_NV12=1 } PixelFormat;

typedef struct { float fx, fy, cx, cy, k1, k2, p1, p2, k3; } CameraIntrinsics; // extend if needed

typedef struct { float R[9]; float t[3]; } CameraExtrinsics; // body<-cam

typedef struct {
  uint64_t t_ns; // sensor time (ns)
  int w,h,stride; PixelFormat fmt;
  CameraIntrinsics K; CameraExtrinsics T_bw;
  uint8_t* data; // producer-owned; consumer copies if needed
} Frame;

typedef struct { Frame left; Frame right; uint64_t t_sync_ns; } StereoFrame;

typedef enum { CONE_LEFT=0, CONE_RIGHT=1, CONE_UNKNOWN=2 } ConeSide;

typedef struct {
  float p_B[3]; // (x,y,z) in body frame
  ConeSide side;
  float conf;
  float cov[6]; // optional, 0 if unused: (xx,xy,xz,yy,yz,zz)
} ConeDet;

typedef struct { uint64_t t_ns; int n; ConeDet dets[512]; } Detections; // fixed upper bound

typedef struct {
  uint64_t t_ns; float x,y,yaw, vx,vy, yaw_rate, ax,ay, steer_rad;
} VehicleState;

typedef struct { float steer_rad; float throttle; float brake; uint64_t t_ns; } ControlCmd;

typedef struct { uint32_t id; uint8_t dlc; uint8_t data[8]; uint64_t t_ns; } CanMsg;
```

---

## Module APIs

**Vision API**

```c
int Vision_Init(const char* config_yaml);
int Vision_ProcessStereo(const StereoFrame* in, Detections* out); // 1=ok, 0=invalid
void Vision_Shutdown(void);
```

**Control API**

```c
int Control_Init(const char* config_yaml);
int Control_Tick(const VehicleState* s, const Detections* dets, ControlCmd* u);
void Control_Shutdown(void);
```

**Stereo Sources** (Sim and Real)

```c
typedef struct IStereoSource {
  void* impl;
  int  (*start)(void* impl);
  int  (*grab)(void* impl, StereoFrame* out); // non-blocking; 1 if frame
  void (*stop)(void* impl);
} IStereoSource;
```

**CAN interface**

```c
int CAN_SendControl(const ControlCmd* u);    // packs via API specification
int CAN_Poll(CanMsg* out);                   // non-blocking RX
```

**Record/Replay**

```c
void Rec_Open(const char* path);
void Rec_Stereo(const StereoFrame*);
void Rec_Detections(const Detections*);
void Rec_StateGT(const VehicleState*);
void Rec_StateEst(const VehicleState*);
void Rec_Cmd(const ControlCmd*);
void Rec_CAN(const CanMsg*);
void Rec_Close(void);
```

---

## Minimal 3D Renderer Plan (Sim Stereo)

**Goal:** Deliver synchronized Left/Right images with known `K`, `T_bw`, and baseline `B`.

**Steps**

1. Add an **OpenGL 3.3 Core** context with SDL2 (keep your SDL2 windowing).

2. Implement a simple forward renderer: ground plane + cone mesh (single draw call instanced), car proxy for sanity.

3. Create two camera rigs: `CamL`, `CamR` with baseline `B` and shared extrinsics origin in body frame.

4. Render to two **FBOs** at the real resolution; **readback** to CPU (PBO for async read).

5. Populate `Frame` structs (RGB24 or NV12). Fill `K`, `T_bw` from config.

6. Add optional artifacts: Gaussian noise, motion blur toggle, exposure jitter, lens distortion.

7. Plug into `IStereoSource` and `Rec_Stereo()`; confirm sync skew ≤ 1 ms.

**Do not overbuild**. One shader, one mesh, one light is enough. Consistency > photorealism.

---

## Control: Cones → Track Details

- Centerline = list of midpoints of left/right cones.

---

## Mode Switching (Sim ↔ Real)

- **Cameras:** swap `IStereoSource` impl; intrinsics/extrinsics supplied via YAML; Vision must not assume fixed values.

- **Actuation:** Sim: `ControlCmd` applies to dynamic model. Real: `ControlCmd` → CAN via generated DBC packers.

- **Health:** after any swap, drop the first 10 frames; Vision rejects if calibration hash changes unexpectedly.

---

## CI & Tests

- **Unit:** math, pack/unpack, queue, splines, triangulation.

- **SIL:** replay logs → identical `Detections`/`ControlCmd` within tolerances.

- **Perf:** enforce budgets per module on CI rig; fail the build if exceeded.

- **HIL:** CAN loopback; verify bus utilization and deadlines.

---

## Hard Truths (don’t ignore)

- 2D SDL plots won’t cut it for Vision. Implement the minimal 3D renderer or you won’t be able to validate.

- Python in the hot loop will introduce jitter. Prototype there, but deploy inference in C++.

- Without synchronized stereo and trustworthy calibration, your depth is fiction. Get hardware sync or enforce tight skew.

- No DBC? No car. Generate packers/unpackers and keep them versioned.

---

# Key Design Decisions (per sub‑team)

## Vision

- **Stereo first**: all prod pipelines assume synchronized Left/Right. If mono is used for a test, the publisher still emits a `StereoFrame` with the missing eye flagged `w=0,h=0` and a status bit.

- **Depth from geometry**: triangulation uses calibrated rigs; learned monocular depth is allowed only as a prior.

- **Latency cap**: ≤ 20 ms E2E @ 1280×720. Anything slower must down‑res or prune the model; Python prototypes are for offline only.

- **Color/side**: prefer side classification from color when reliable; fall back to learned side or UNKNOWN.

- **Deterministic ABI**: emit fixed‑size `Detections` block; no dynamic allocations or variable schemas.

## Control

- **Track built here**: cones → boundaries → centerline happen inside Control, not Vision.

- **200 Hz loop**: lateral + longitudinal controllers at 200 Hz, planner at ≥ 100 Hz, synchronized to the same clock.

- **Forward prediction**: compensate for perception latency by projecting `VehicleState` to `t_now + τ_vision`.

- **Command schema**: `ControlCmd{ steer_rad, throttle, t_ns }` **required**. `brake` is **optional**; if present in DBC, include it. On‑car, both channels should exist even if zero.

- **Fail‑safe**: stale or invalid detections → decay speed to zero within 200 ms and hold steering near center (rate‑limited).

## Simulation/Integration

- **Minimal 3D**: single OpenGL forward pipeline with instanced cones; realism is secondary to calibration fidelity.

- **Hot‑swap**: atomic swap between `SimStereoSource` and `RealStereoSource`; drop first 10 frames after swap.

- **Single clock**: all timestamps monotonic ns; renderer and capture report sensor‑time.

- **Record/replay by default**: every run produces a log; regression and SIL consume the same.

---

# Interface Section (canonical)

## Message Schemas

- **StereoFrame**

  - `left: Frame`, `right: Frame`, `t_sync_ns`.

  - **Resolution (default)**: 1280×720 (16:9). **Frame rate**: 60 Hz (configurable). **Pixel format**: `RGB24` **or** `NV12`. Pipelines must accept both.

  - **FoV**: 70–90° horizontal (config). **Baseline**: 0.20–0.30 m (config). **Shutter**: global preferred; if rolling, include `readout_us` in config.

  - **Calibration**: each `Frame` carries `K` and `T_bw`. The stereo baseline is implied by `T_bw` of left and right.

  - **Sync**: |`left.t_ns`−`right.t_ns`| ≤ 1 ms; else Vision returns `INVALID_SYNC`.

- **Detections**

  - Up to 512 cones: `p_B=(x,y,z) [m]`, `side∈{LEFT,RIGHT,UNKNOWN}`, `conf∈[0,1]`, optional `cov[6]`.

- **VehicleState**

  - `x,y,yaw,vx,vy,yaw_rate,ax,ay,steer_rad,t_ns` (extend as needed; estimator owns its schema version).

- **ControlCmd**

  - Required: `steer_rad, throttle, t_ns`. Optional: `brake` (defaults to 0 if omitted). All CAN mappings come from DBC.

## C APIs (stable)

```c
// Vision
int Vision_Init(const char* config_yaml);
int Vision_ProcessStereo(const StereoFrame* in, ConeDet* out); // 1=OK, 0=INVALID_*; see error codes
void Vision_Shutdown(void);

// Control
int Control_Init(const char* config_yaml);
int Control_Tick(const VehicleState* s, const ConeDet* dets, ControlCmd* u); // 1=OK, 0=HOLD
void Control_Shutdown(void);

// Stereo Sources (Sim/Real)
typedef struct IStereoSource {
  void* impl; int (*start)(void*);
  int (*grab)(void*, StereoFrame*); // non-blocking; 1 if frame available
  void (*stop)(void*);
} IStereoSource;

// CAN (on-car)
int CAN_SendControl(const ControlCmd* u);    // packs via generated DBC
int CAN_Poll(CanMsg* out);                   // non-blocking RX
```

## Return/Status Codes

- `OK=1` — valid output.

- `HOLD=0` — (Control) keep last safe cmd.

- Vision extended codes (log only): `INVALID_SYNC`, `INVALID_CALIB`, `NO_CONES`, `INFERENCE_TIMEOUT`.

## Config & Calibration

- YAML keys for cameras: `resolution`, `fps`, `pixel_format`, `fx,fy,cx,cy`, `distortion:{k1,k2,p1,p2,k3}`, `T_bw`, `readout_us`, `calib_version`.

- All modules must log the loaded calibration hash and version on startup.

---

# Detailed Specs for `StereoFrame` (fill‑ins resolved)

- **Resolution**: default 1280×720; allowable set {640×480, 1280×720, 1920×1080}. All training and runtime must support at least 1280×720.

- **Aspect ratio**: 4:3 or 16:9; team standard is **16:9**.

- **Pixel format**: `RGB24` (sim default) and `NV12` (camera default). Vision preproc must normalize to model input.

- **FPS**: default 60; acceptable 30–90; configure per hardware.

- **FoV**: 80°±10° horizontal (document actual in YAML).

- **Baseline**: 0.24 m default; document exact measured value.

- **Exposure/Sync**: hardware trigger preferred; software sync allowed if skew < 1 ms (enforced).

---

# Addenda v1.2 — FS‑AI‑aligned details, interfaces, safety & mode switching

## What’s new in this addendum

- Filled **budgets** with concrete values; added **Vision/Control acceptance** gates.

- Added a **Safety & Compliance** block (RES/EBS, off‑track, cone collision monitors).

- Defined a **Mode & Handshake** for `--mode={sim|hil|car}` and ECU connectivity checks.

- Expanded **Interfaces** with CAN pacing and status semantics.

- Clarified **StereoFrame specs** for ZED 2i and confirmation steps.

## Concrete Budgets (cross‑team)

- Vision end‑to‑end (frame → detections): **≤ 20 ms** @ 1280×720; drop rate **< 1%**.

- Planner + Controllers combined tick: **≤ 2 ms**.

- Renderer (3D) at 60 Hz: **≤ 6 ms** per frame.

- CAN send deadline: **≤ 5 ms** from `ControlCmd.t_ns` to bus.

## Vision — Acceptance (ship gates)

- **Accuracy:** mAP@0.5 ≥ **0.90** (sim), ≥ **0.80** (early real); depth RMSE ≤ **0.25 m** to **30 m**.

- **Timing:** E2E ≤ **20 ms** @ 720p; sustained run 10 min with <1% drops.

- **Robustness:** tolerate **≤ 1 ms** L/R skew (else return `INVALID_SYNC`); handle **10%** cone occlusion with ≤ **0.3 m** boundary error.

## Control — Acceptance

- **SIL:** complete a lap with **0** boundary violations; lap time ≤ **+15%** of baseline.

- **HIL:** CAN timing stable (no ECU timeout), **≤ 30%** bus load at peak; watchdogs trip correctly.

## Safety & Compliance (FS‑AI‑aligned)

- **RES & E‑Stop:** consume RES/EBS status from VCU; any trigger → **FAULT**: torque=0, steering→center (rate‑limited), brake=ON if supported. Log reason + `t_ns`.

- **Off‑track:** signed distance to boundaries; if > **0.5 m** outside for **>200 ms**, controlled stop + flag.

- **Cone collision:** footprint intersection or min distance < **0.2 m** with decreasing trend → slow to crawl or stop.

- **Gating:** do not arm traction until Ready/Go and RES safe; require N good CAN frames (e.g., **10 in 200 ms**).

- **State machine:** `BOOT → INIT → MAPPING → RACING → FAULT (sticky) → SAFE_SHUTDOWN`.

## Mode & Handshake (sim ↔ HIL ↔ car)

- Runtime flag: `--mode={sim|hil|car}`

  - **sim:** no physical CAN; use simulated VCU; renderer on.

  - **hil:** SocketCAN on `vcan0` or loopback device; full ECU handshake; vehicle static/dyno.

  - **car:** physical CAN; renderer optional for HUD.

- **ECU handshake (ADS‑DV / DBC):**

  1. Open CAN + init API/wrapper.

  2. Start **periodic tx every 10 ms** (ai2vcu). Internal ECU timeout if slower; don’t exceed ~8 ms min period.

  3. Continuously read vcu2ai; require Ready/Go, RES=SAFE, EBS OK before movement.

  4. Mark link **UP** only after N consecutive good frames.

  5. On RES/EBS or link loss → **FAULT** immediately.

## Interfaces — Clarifications

- **CAN pacing:** call the "AI→VCU set" function at **10 ms** cadence; burst‑limit to avoid bus overload. Read "VCU→AI" asynchronously; expose counters.

- **Health topics:** `ecu_link`, `res_state`, `ebs_state`, `can_rx_rate`, `ai_tx_age_ms`, `vision_latency_ms` — all in HUD and logs.

- **Status codes additions:** Vision may log `MODEL_HASH_MISMATCH`, `BAD_CALIB_VERSION`.

## StereoFrame — confirmations (ZED 2i)

- Factory baseline **120 mm**; verify via calibration; FoV ~**110° diag** (confirm lens in metadata). Keep YAML authoritative.

## Mapping → Racing handover

- **Lap 1 (mapping):** feed a small rolling horizon of cones/centerline; run controller with conservative gains and speed caps.

- **Green‑light:** after loop closure / track complete, `Control` raises `mode=RACING`; planner switches to full‑track lookahead.

## Off‑track & Cone‑hit implementation sketch

- Maintain vehicle polygon in body frame; compute min distance to each cone and to fitted boundaries each tick (200 Hz). Use hysteresis windows to avoid chatter.

## Mode flag example

- CLI: `fsai_run --mode=sim --can=off`

- Config: `mode: sim|hil|car`, `can_iface: can0|vcan0`, `dbc: adsdv_2021.dbc`, `ecu: ads-dv|custom`.

---

---

# v1.3 — Interim Plan: Visionless Bootstrapping with **Fake Sub‑Systems**

## Why

Vision is not ready. We still want the rest of the stack (control, safety, CAN, logs, modes) to mature **now**. This plan adds **drop‑in fake providers** that publish the _same_ interfaces your real modules will, so you can swap them out without touching integration code.

## What changed vs v1.2

- Removed references to PID/LQR/PP in the baseline. The baseline controller is your **Custom Lookahead Controller (CLC)**.

- Added **FakeVision**, **FakePlanner**, **FakeEstimator**, **FakeCAN** modules with clear deprecation paths.

- Added an **Adapter** that turns cones/checkpoints into the current CLC inputs.

- Added a **Controller Audit** of your code with concrete fixes.

---

## Provider Registry (runtime‑switchable)

A simple name→vtable registry; each interface has multiple providers. Configure via YAML or `--providers.*` flags.

```
providers:
  vision: fake_from_sim   # fake_from_sim | zed | sim_stereo
  planner: fake_track    # fake_track | spline
  estimator: fake_truth  # fake_truth | ekf
  can: fake              # fake | dbc
  mode: sim              # sim | hil | car
```

### Interfaces (unchanged)

- `int Vision_ProcessStereo(const StereoFrame*, Detections*)`

- `int Control_Tick(const VehicleState*, const Detections*, ControlCmd*)`

- `int CAN_SendControl(const ControlCmd*)`, `int CAN_Poll(CanMsg*)`

- `StereoSource.grab(StereoFrame*)` (may be fake/no‑op)

### Fake providers (drop‑in)

- **FakeVision (fake_from_sim)**

  - **Input:** (ignored for now) `StereoFrame` or `NULL`.

  - **Output:** `Detections` built **directly from PathLogic/track_generator** cones (ground truth) with optional noise/dropouts:

    - Gaussian noise σ_pos = 0.05–0.10 m; 5–10% random dropout; side labels from track metadata.

    - Timestamps from the shared clock; honors `t_sync_ns` if present.

  - **Deprecation:** replaced by `zed` provider when ready.

- **FakePlanner (fake_track)**

  - **Input:** `Detections` or (if missing) PathLogic centerline.

  - **Output to CLC:** a **checkpoint array** (your current API) derived from centerline (fixed spacing 0.5–1.0 m, forward‑looking horizon 60–80 m). No speed profile.

  - **Deprecation:** replaced by `spline` planner that fits boundaries/centerline from real cones.

- **FakeEstimator (fake_truth)**

  - **Input:** sim truth state.

  - **Output:** `VehicleState` identical to truth; optional noise.

  - **Deprecation:** replaced by `ekf` (IMU + wheels + steering + optional VO) on HIL/car.

- **FakeCAN (fake)**

  - **Send:** just logs `ControlCmd` with timestamps and bus‑util estimates; mirrors into log channel `can_tx`.

  - **Poll:** synthesizes a minimal `vcu2ai` heartbeat (RES=SAFE, Ready=1) for handshake testing.

  - **Deprecation:** replaced by `dbc` provider or FS‑AI API wrapper.

---

## Adapter — make your controller the baseline

Your **Custom Lookahead Controller (CLC)** consumes a **vector of checkpoints**. Until Vision+Planner are live, the Adapter builds this from:

1. **FakePlanner** (centerline from PathLogic), or

2. Real **Detections** (later) → assign sides → fit boundaries → compute centerline → sample checkpoints.

**Adapter API**

```c
// Builds checkpointPositions[] for CLC from either Detections or PathLogic truth.
int Adapter_BuildCheckpoints(const Detections* dets_or_null,
                             const PathMeta* path_or_null,
                             Vector3* checkpointPositions,
                             int maxN,
                             float sample_m,
                             float horizon_m);
```

- Guarantees fixed spacing and forward ordering; drops any back‑facing points.

- Returns `N` and a `status` bitfield (e.g., `AD_NO_LEFT`, `AD_NO_RIGHT`, `AD_LOW_DENSITY`).

**Control_Tick wiring (now)**

```
VehicleState s = Estimator();
Vector3 cps[1024]; int N = Adapter_BuildCheckpoints(fake_dets, path, cps, 1024, 0.75f, 80.0f);
float u_throttle = Controller_GetThrottleInput(cps, N, s.vx_forward, &pose, &cfg, dt);
float u_steer    = Controller_GetSteeringInput  (cps, N, s.vx_forward, &pose, &cfg, dt);
pack ControlCmd and send/log per budgets
```

---

## Controller Audit (your code) — problems & fixes

### Problems (technical)

1. **Unit ambiguity**: `Vector2_SignedAngle` likely returns **radians** or **degrees**; `MAX_ANGLE=21.0f` suggests degrees. Mixed units will wreck steering/throttle.

2. **Throttle model is brittle**: `expectedSpeed = 1/(accelFactor * angle)` has a singularity at 0 and grows unbounded; it ignores curvature and friction limits.

3. **`accelerationNeeded` hack**: forcing `1.0f` when near zero introduces steps; also the division `distanceToCheckpoint / speed` explodes for small speeds.

4. **VLAs on stack**: `cachedDirections[lookaheadMax+1]` uses C99 VLAs; large `lookaheadMax` → stack spikes; non‑portable in C++.

5. **No NaN guards**: `Vector2_Normalize` with near‑zero `diff` can yield NaNs; same for `distanceToCheckpoint=0`.

6. **`printf` in hot path**: `printf("LIObj...")` every tick kills determinism and latency.

7. **Lookahead index formula**: `floor(sensitivity*velocity)+1` ties indices to raw m/s and checkpoint density implicitly; not robust across tracks/horizons.

8. **Angle linearization**: `angle/ MAX_ANGLE` ignores wheelbase, speed, and geometry; steering should depend on lateral error & heading error together.

9. **No rate limits**: steering and throttle outputs can jump; no slew‑rate limiting or saturation smoothing.

10. **Sign conventions**: `return steeringInput` vs `-steeringInput` comment mismatch; likely a frame/sign bug.

11. **Magic constants**: `MAX_SPEED`, `MAX_ACC`, `MAX_ANGLE` are hardcoded without matching vehicle params.

12. **No fail‑safes**: no stale data handling, no bounds checking for `nCheckpoints`, no watchdog on dt.

### Fixes (minimal, keep architecture)

- **Unify units**: decide **radians** everywhere; set `MAX_STEER_RAD` from vehicle.

- **Robust lookahead**: compute **metric** lookahead distance `L_d = k0 + k1*v` (meters), then pick the index whose arc‑length ≈ `L_d`.

- **Guard math**: if `distanceToCheckpoint < 0.2 m` → skip/advance; clamp `speed` lower bound but never inject `1.0f`.

- **Replace VLA** with fixed ring buffers sized to horizon points; reuse across ticks.

- **Throttle model**: base on **target speed profile** (even a crude one: `v_target = clamp(v0 + k*|angle|^-1, v_max_by_curvature)`), or simpler: proportional to `(v_target - v_now)` with capped accel/decel.

- **Slew‑rate limiters**: apply first‑order filter or explicit `Δu_max/dt` on steer and throttle.

- **Remove `printf`**; push to a sampled telemetry channel.

- **Sign test**: validate steering sign with a unit test (car placed left of a straight line should steer right, etc.).

- **Bounds**: check `nCheckpoints>0`, `lookaheadIndex>=0`, `lookaheadIndex<N`.

---

## Safety even with fakes

- **Watchdogs**: camera source alive (or fake), ECU link alive (or fake), `vision_latency_ms`, `ai_tx_age_ms`, controller deadline. Any red → **HOLD** and decay to zero within 200 ms.

- **Off‑track** (sim): compute signed distance to known boundaries from PathLogic; enforce stop rules to exercise the logic now.

- **Cone‑hit monitor** (sim): footprint overlap with ground‑truth cones to test the stop path.

---

## Deprecation path (kill switches)

- `vision=fake_from_sim` → `vision=zed` once stereo is ready.

- `planner=fake_track` → `planner=spline` once boundary fit lands.

- `estimator=fake_truth` → `estimator=ekf` on HIL/car.

- `can=fake` → `can=dbc` (or fs‑ai API) when you wire the real ECU.

Each provider must implement a **self‑test** and log `provider_name`, `version`, and `calib_hash` on startup.

---

## TODOs you can implement this week (fast wins)

- Add **Provider Registry** and flags/env to select providers.

- Implement **FakeVision** (PathLogic → Detections with noise/dropout) + Adapter.

- Implement **FakeCAN** and ECU handshake simulator; enforce **10 ms** pacing in the scheduler.

- Add **watchdogs/HUD** counters (`ecu_link`, `ai_tx_age_ms`, `vision_latency_ms`).

- Replace VLA with fixed buffers; remove `printf` spam; add sign/unit tests for steering.

- Hook **record/replay** for `detections`, `state_gt`, `cmd`, `can_tx/can_rx` even in fake modes.

---
