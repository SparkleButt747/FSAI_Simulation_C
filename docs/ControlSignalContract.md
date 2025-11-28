# Control Signal Contract

These ranges are enforced by `ControlSignalAdapter` and the surrounding
simulation glue. Values outside the contract are clamped and logged as
errors so downstream subsystems (CAN/Velox/GUI) can surface the fault.

## Inputs from AI / RA controller
- `throttle` (normalized): **[0, 1]**. Values < 0 are treated as 0.0. Represents drive request only (no regen).
- `brake` (normalized): **[0, 1]**. Values < 0 are treated as 0.0.
- `steer_rad`: **[-0.3665, 0.3665] rad** (±21°). Anything outside is clamped before sending to CAN or Velox.
- `t_ns`: monotonic nanosecond timestamp used for staleness checks.

## Derived commands
- CAN torque request:
  - Front axle: **[0, 195] Nm** (DBC `kMaxAxleTorqueNm`)
  - Rear axle: **[0, 195] Nm** (DBC `kMaxAxleTorqueNm`)
  - Fractions must sum to > 0; both axles cannot be configured to 0 Nm.
- Velox command:
  - Accepts normalized throttle/brake **[0, 1]** or axle torques converted to normalized drive/brake using the configured maxima above.
  - Steering is clamped to the same ±21° window before being mapped into Velox’s steering controller.

## Error handling
- Non‑finite or out‑of‑range inputs are rejected and clamped with a descriptive log entry.
- Stale commands (older than the configured freshness window) emit a warning and do not override manual inputs.
- Misconfigured torque fractions (zero sum) or zero axle maxima fall back to safe defaults and emit errors visible in the GUI log stream.
