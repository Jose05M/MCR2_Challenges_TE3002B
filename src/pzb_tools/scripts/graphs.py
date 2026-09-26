#!/usr/bin/env python3

import sys

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# =========================================================
# LOAD CSV
# =========================================================

# Usage: python3 graphs.py [robot_analysis.csv]
df = pd.read_csv(sys.argv[1] if len(sys.argv) > 1 else "robot_analysis.csv")

# =========================================================
# GET DATA
# =========================================================

t = df['time'].values
x = df['x'].values
y = df['y'].values
v = df['v'].values
w = df['w'].values

# =========================================================
# DESIRED TARGET / WAYPOINT
# =========================================================

targets = [
    (1.5, 0.0),
    (1.5, 1.5),
    (0.0, 1.5),
    (0.0, 0.0)
]

# =========================================================
# POSITION ERROR
# =========================================================

error = []

current_target = 0

target_threshold = 0.15

for i in range(len(x)):

    xd, yd = targets[current_target]

    e = np.sqrt((xd - x[i])**2 + (yd - y[i])**2)

    error.append(e)

    # Cambiar al siguiente waypoint
    if e < target_threshold:

        if current_target < len(targets) - 1:
            current_target += 1

error = np.array(error)

# =========================================================
# RMS ERROR
# =========================================================

rms_error = np.sqrt(np.mean(error**2))

# =========================================================
# MAX ERROR
# =========================================================

max_error = np.max(error)

# =========================================================
# FINAL ERROR
# =========================================================

final_error = error[-1]

# =========================================================
# TOTAL DISTANCE TRAVELED
# =========================================================

dx = np.diff(x)
dy = np.diff(y)

distance = np.sum(np.sqrt(dx**2 + dy**2))

# =========================================================
# TOTAL TIME
# =========================================================

total_time = t[-1]

# =========================================================
# AVERAGE VELOCITIES
# =========================================================

avg_v = np.mean(np.abs(v))
avg_w = np.mean(np.abs(w))

# =========================================================
# CONTROL SMOOTHNESS
# =========================================================

dv = np.diff(v)
dw = np.diff(w)

smoothness_v = np.mean(np.abs(dv))
smoothness_w = np.mean(np.abs(dw))

# =========================================================
# PRINT METRICS
# =========================================================

print("\n================ METRICS ================\n")

print(f"RMS Error              : {rms_error:.4f} m")
print(f"Maximum Error          : {max_error:.4f} m")
print(f"Final Error            : {final_error:.4f} m")

print(f"\nTotal Distance         : {distance:.4f} m")
print(f"Total Time             : {total_time:.4f} s")

print(f"\nAverage Linear Vel     : {avg_v:.4f} m/s")
print(f"Average Angular Vel    : {avg_w:.4f} rad/s")

print(f"\nLinear Smoothness      : {smoothness_v:.4f}")
print(f"Angular Smoothness     : {smoothness_w:.4f}")

print("\n=========================================\n")

# =========================================================
# PLOTS
# =========================================================

# ---------------------------------------------------------
# 1. TRAJECTORY
# ---------------------------------------------------------

plt.figure(figsize=(8, 6))

# Trayectoria real
plt.plot(x, y, label='Robot Trajectory')

# Waypoints
for i, (tx, ty) in enumerate(targets):

    if i == 0:
        plt.scatter(
            tx,
            ty,
            marker='x',
            s=120,
            label='Targets'
        )
    else:
        plt.scatter(
            tx,
            ty,
            marker='x',
            s=120
        )

plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Robot Trajectory')

plt.grid(True)
plt.legend()

# ---------------------------------------------------------
# 2. POSITION ERROR
# ---------------------------------------------------------

plt.figure(figsize=(8, 6))

plt.plot(t, error)

plt.xlabel('Time [s]')
plt.ylabel('Error [m]')
plt.title('Position Error vs Time')

plt.grid(True)

# ---------------------------------------------------------
# 3. LINEAR VELOCITY
# ---------------------------------------------------------

plt.figure(figsize=(8, 6))

plt.plot(t, v)

plt.xlabel('Time [s]')
plt.ylabel('v [m/s]')
plt.title('Linear Velocity')

plt.grid(True)

# ---------------------------------------------------------
# 4. ANGULAR VELOCITY
# ---------------------------------------------------------

plt.figure(figsize=(8, 6))

plt.plot(t, w)

plt.xlabel('Time [s]')
plt.ylabel('w [rad/s]')
plt.title('Angular Velocity')

plt.grid(True)

# ---------------------------------------------------------
# 5. CONTROL SMOOTHNESS
# ---------------------------------------------------------

plt.figure(figsize=(8, 6))

plt.plot(dv, label='Δv')
plt.plot(dw, label='Δw')

plt.xlabel('Samples')
plt.ylabel('Control Change')
plt.title('Control Smoothness')

plt.grid(True)
plt.legend()

# =========================================================
# SAVE FIGURES
# =========================================================

plt.figure(1)
plt.savefig('trajectory.png')

plt.figure(2)
plt.savefig('position_error.png')

plt.figure(3)
plt.savefig('linear_velocity.png')

plt.figure(4)
plt.savefig('angular_velocity.png')

plt.figure(5)
plt.savefig('control_smoothness.png')

# =========================================================
# SHOW ALL
# =========================================================

plt.show()
