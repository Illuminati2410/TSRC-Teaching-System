import pandas as pd
import numpy as np

# ==========================================
# SETTINGS
# ==========================================

INPUT_FILE = "tracker_data.csv"
OUTPUT_FILE = "fairino_path.txt"

SCALE = 0.5

START_X = 350
START_Y = -450
START_Z = 450

RX = 180.0
RY = 0.0
RZ = 90.0

STEP = 20

# Maximum allowed jump between saved points (mm)
MAX_JUMP = 150

# ==========================================
# LOAD DATA
# ==========================================

df = pd.read_csv(INPUT_FILE)

x = df["x"].to_numpy() * 1000
y = df["y"].to_numpy() * 1000
z = df["z"].to_numpy() * 1000

# ==========================================
# TRAJECTORY CENTER
# ==========================================

cx = np.median(x)
cy = np.median(y)
cz = np.median(z)

print("\nTracker Center")
print(f"CX = {cx:.1f}")
print(f"CY = {cy:.1f}")
print(f"CZ = {cz:.1f}")

# ==========================================
# RAW TRACKER LIMITS
# ==========================================

print("\nRaw Tracker Limits")

print(f"X: {x.min():.1f} -> {x.max():.1f}")
print(f"Y: {y.min():.1f} -> {y.max():.1f}")
print(f"Z: {z.min():.1f} -> {z.max():.1f}")

print("\nTracker Start")
print(f"{x[0]:.1f}, {y[0]:.1f}, {z[0]:.1f}")

print("\nTracker End")
print(f"{x[-1]:.1f}, {y[-1]:.1f}, {z[-1]:.1f}")

# ==========================================
# TRANSFORM TO ROBOT SPACE
# ==========================================

robot_x = (x - cx) * SCALE + START_X
robot_y = (y - cy) * SCALE + START_Y
robot_z = (z - cz) * SCALE + START_Z

robot_x = np.clip(robot_x, 150, 550)
robot_y = np.clip(robot_y, -700, -200)
robot_z = np.clip(robot_z, 250, 650)

# ==========================================
# REPORT
# ==========================================

print("\nTracker Range")

print(f"X span = {x.max()-x.min():.1f} mm")
print(f"Y span = {y.max()-y.min():.1f} mm")
print(f"Z span = {z.max()-z.min():.1f} mm")

print(f"\nUsing scale = {SCALE}")

print("\nRobot Limits")

print(f"X: {robot_x.min():.1f} -> {robot_x.max():.1f}")
print(f"Y: {robot_y.min():.1f} -> {robot_y.max():.1f}")
print(f"Z: {robot_z.min():.1f} -> {robot_z.max():.1f}")

# ==========================================
# CHECK FOR LARGE JUMPS
# ==========================================

print("\nChecking trajectory for large jumps...")

jump_count = 0

for i in range(1, len(robot_x)):

    dx = abs(robot_x[i] - robot_x[i - 1])
    dy = abs(robot_y[i] - robot_y[i - 1])
    dz = abs(robot_z[i] - robot_z[i - 1])

    if dx > MAX_JUMP or dy > MAX_JUMP or dz > MAX_JUMP:

        jump_count += 1

        print(f"\nJUMP #{jump_count}")

        print(
            f"Prev: "
            f"{robot_x[i-1]:.1f}, "
            f"{robot_y[i-1]:.1f}, "
            f"{robot_z[i-1]:.1f}"
        )

        print(
            f"Curr: "
            f"{robot_x[i]:.1f}, "
            f"{robot_y[i]:.1f}, "
            f"{robot_z[i]:.1f}"
        )

print(f"\nTotal jumps found = {jump_count}")

# ==========================================
# SAVE FAIRINO FILE
# ==========================================

print("\nWriting trajectory...")
print("STEP =", STEP)

count = 0
skipped = 0

last_px = None
last_py = None
last_pz = None

with open(OUTPUT_FILE, "w") as f:

    for i in range(0, len(robot_x), STEP):

        px = robot_x[i]
        py = robot_y[i]
        pz = robot_z[i]

        if last_px is not None:

            dx = abs(px - last_px)
            dy = abs(py - last_py)
            dz = abs(pz - last_pz)

            if dx > MAX_JUMP or dy > MAX_JUMP or dz > MAX_JUMP:

                skipped += 1

                print(
                    f"SKIPPED JUMP "
                    f"dx={dx:.1f} "
                    f"dy={dy:.1f} "
                    f"dz={dz:.1f}"
                )

                continue

        f.write(
            f"{px:.3f} "
            f"{py:.3f} "
            f"{pz:.3f} "
            f"{RX:.3f} "
            f"{RY:.3f} "
            f"{RZ:.3f}\n"
        )

        last_px = px
        last_py = py
        last_pz = pz

        count += 1

print("len(robot_x) =", len(robot_x))
print("STEP =", STEP)
print("Expected points =", len(robot_x)//STEP)

print("\nPoints written =", count)
print("Points skipped =", skipped)

print(f"\nSaved: {OUTPUT_FILE}")

print("\nFirst generated pose")

print(
    f"{robot_x[0]:.3f} "
    f"{robot_y[0]:.3f} "
    f"{robot_z[0]:.3f} "
    f"{RX:.3f} "
    f"{RY:.3f} "
    f"{RZ:.3f}"
)

import os

print(
    "\nFile size:",
    os.path.getsize(OUTPUT_FILE),
    "bytes"
)