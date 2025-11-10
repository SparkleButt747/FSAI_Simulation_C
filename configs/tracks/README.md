# Track CSV Schema

The simulation ships with predefined cone layouts for the acceleration and skidpad
missions. Track geometry is stored in comma-separated value (CSV) files located in
this directory.

Each non-comment row in a CSV file must contain exactly five columns:

1. `type` – element classification (`start`, `left`, `right`, or `checkpoint`).
2. `x_m` – X position in **meters** in the simulation ground plane.
3. `y_m` – Y position in **meters** (height above ground, typically `0`).
4. `z_m` – Z position in **meters** in the simulation ground plane.
5. `yaw_deg` – Heading about the up axis in **degrees**.

Lines beginning with `#` or blank lines are ignored. The loader converts the yaw
value to radians and builds `TrackResult`/`TrackData` objects consumed by the
simulator.

The provided assets are:

- `acceleration.csv` – straight-line acceleration mission.
- `skidpad.csv` – circular skidpad mission.

Custom layouts can be created by following the same schema.
