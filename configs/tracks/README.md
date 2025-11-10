# Track CSV Schema

The simulation ships with predefined cone layouts for the acceleration and skidpad
missions. Track geometry is stored in comma-separated value (CSV) files located in
this directory.

Each non-comment row in a CSV file must contain exactly seven columns:

1. `tag` – semantic classification of the element.
2. `x` – X coordinate in **meters** in the simulator ground plane.
3. `y` – Z coordinate (forward axis) in **meters** in the simulator ground plane.
4. `direction` – heading about the up axis in **radians**.
5. `x_variance` – measurement variance in the X direction (unused by the simulator, kept for traceability).
6. `y_variance` – measurement variance in the Z direction (unused by the simulator).
7. `xy_covariance` – covariance term between X and Z (unused by the simulator).

Lines beginning with `#` or blank lines are ignored. The loader consumes the
`tag` column as follows:

- `blue` – left-hand cones.
- `yellow` – right-hand cones.
- `big_orange` – start line cones.
- `orange` – pointer/finish cones (sorted to the left or right based on their lateral sign).
- `midpoint` – explicit checkpoints along the track centreline.
- `car_start` – starting position of the vehicle (inserted as the first checkpoint).

If no `midpoint` entries are provided, checkpoints are derived from the left and
right boundaries by pairing the closest cones.

The provided assets are:

- `acceleration.csv` – straight-line acceleration mission.
- `skidpad.csv` – circular skidpad mission.

Custom layouts can be created by following the same schema.
