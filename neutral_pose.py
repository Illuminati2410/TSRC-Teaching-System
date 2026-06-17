import os

OUTPUT_FILE = "neutral_pose_test.txt"

# ==========================================
# SAFE HOME POSE
# ==========================================

HOME_X = 300
HOME_Y = -500
HOME_Z = 450

HOME_R = 180
HOME_P = 0
HOME_YAW = 90

# ==========================================
# CANDIDATE NEUTRAL POSE
# ==========================================

NX = 100
NY = -400
NZ = 300

R = 180
P = 0
YAW = 90

# Test distance
D = 172

# Number of interpolation points
INTERP = 10

# ==========================================
# HELPER
# ==========================================

points = []


def add_line(start, end, steps=10):

    x1, y1, z1 = start
    x2, y2, z2 = end

    for i in range(1, steps + 1):

        t = i / steps

        x = x1 + (x2 - x1) * t
        y = y1 + (y2 - y1) * t
        z = z1 + (z2 - z1) * t

        points.append((x, y, z))


# ==========================================
# BUILD TRAJECTORY
# ==========================================

home = (HOME_X, HOME_Y, HOME_Z)
neutral = (NX, NY, NZ)

# Start at home
points.append(home)

# Home -> Neutral
add_line(home, neutral, INTERP)

# ------------------
# +X
# ------------------

px = (NX + D, NY, NZ)

add_line(neutral, px, INTERP)
add_line(px, neutral, INTERP)

# ------------------
# -X
# ------------------

mx = (NX - D, NY, NZ)

add_line(neutral, mx, INTERP)
add_line(mx, neutral, INTERP)

# ------------------
# +Y
# ------------------

py = (NX, NY + D, NZ)

add_line(neutral, py, INTERP)
add_line(py, neutral, INTERP)

# ------------------
# -Y
# ------------------

my = (NX, NY - D, NZ)

add_line(neutral, my, INTERP)
add_line(my, neutral, INTERP)

# ------------------
# +Z
# ------------------

pz = (NX, NY, NZ + D)

add_line(neutral, pz, INTERP)
add_line(pz, neutral, INTERP)

# ------------------
# -Z
# ------------------

mz = (NX, NY, NZ - D)

add_line(neutral, mz, INTERP)
add_line(mz, neutral, INTERP)

# ------------------
# Return Home
# ------------------

add_line(neutral, home, INTERP)

# ==========================================
# SAVE FILE
# ==========================================

with open(OUTPUT_FILE, "w") as f:

    for x, y, z in points:

        f.write(
            f"{x:.3f} "
            f"{y:.3f} "
            f"{z:.3f} "
            f"{R:.3f} "
            f"{P:.3f} "
            f"{YAW:.3f}\n"
        )

print(f"Saved {len(points)} points")
print(f"File: {OUTPUT_FILE}")