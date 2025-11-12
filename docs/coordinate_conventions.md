# Coordinate System & Unit Conventions

This document summarizes the canonical frames and unit conversions used by the
simulator so that the physics models, world/track logic, rendering, and I/O all
remain aligned.

## Frame Overview

| Frame | Axes (x, y, z) | Notes | Primary Users |
| ----- | -------------- | ----- | ------------- |
| **Physics / World** | (forward/east, left/north, up) | Right-handed XY ground plane with Z up. `VehicleState` stores position in this frame while velocities/accelerations are expressed in the vehicle body frame. | Dynamic bicycle model, physics integration, telemetry. |
| **Track / Render** | (east/right, up, north/forward) | Right-handed XZ ground plane used by procedural track generation, checkpoint data, and controller logic. | `World`, racing controller, cone geometry. |
| **SDL Screen** | (right, down, out of screen) | 2D raster coordinates with origin at the top-left. | Stereo display, ImGui overlays. |

The helper `fsai::sim::CoordinateFrames` utilities formalize these frames and
provide functions to translate positions between them.

## Position Conversion

* Use `trackPointToWorldPosition`/`trackXZToWorldXY` to convert track-space
  checkpoint or cone data into physics world coordinates.
* Use `worldStateToTransform` (with an optional height override) to update the
  public `Transform` that feeds the controller and telemetry layers from a
  `VehicleState` produced by the physics model.
* Never manipulate coordinate axes manually—funnel all conversions through the
  helper functions to keep the conventions centralized.

## Orientation & Angles

* All APIs now operate in **radians**. `Vector2_SignedAngle` and the racing
  controller have been updated accordingly, removing implicit degree/radian
  conversions.
* The maximum steering constant and controller acceleration factor were scaled
  to match radian inputs so downstream behaviour is unchanged.

## Practical Guidelines

1. When ingesting external assets (CSV tracks, YAML parameters), keep them in
   the track/render frame until they need to interact with the physics model.
2. When exposing data to UI or CAN emulation layers, convert from the physics
   frame using the helpers to avoid axis swaps.
3. Whenever a new subsystem is introduced (e.g., OpenGL renderer), document the
   frame it expects and add conversion helpers if it does not align with the
   existing frames.
