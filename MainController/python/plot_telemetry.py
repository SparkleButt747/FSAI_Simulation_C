#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

# Load car telemetry CSV.
df = pd.read_csv("/Users/brndy.747/Projects/c_stuff/FSAI_Simulation_C/build/CarStateLog.csv")

# Plot car trajectory: map x vs. z.
plt.figure(figsize=(8,6))
plt.plot(df['x'], df['y'], marker='o', linestyle='-', label="Car Trajectory")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("Car Trajectory")
plt.legend()
plt.grid(True)
plt.savefig("trajectory.png")

# Plot car longitudinal velocity (v_x) over time.
plt.figure(figsize=(8,6))
plt.plot(df['time'], df['v_x'], marker='.', linestyle='-', color='r', label="v_x (Longitudinal Velocity)")
plt.xlabel("Time (s)")
plt.ylabel("v_x (m/s)")
plt.title("Longitudinal Velocity over Time")
plt.legend()
plt.grid(True)
plt.savefig("velocity.png")

plt.show()
