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

NX = 260
NY = -400
NZ = 300

R = 180
P = 0
YAW = 90

D = 50

# ==========================================
# BUILD TEST TRAJECTORY
# ==========================================

points = [

    # Move to neutral
    (HOME_X, HOME_Y, HOME_Z),
    (NX, NY, NZ),

    # +X
    (NX + D, NY, NZ),
    (NX, NY, NZ),

    # -X
    (NX - D, NY, NZ),
    (NX, NY, NZ),

    # +Y
    (NX, NY + D, NZ),
    (NX, NY, NZ),

    # -Y
    (NX, NY - D, NZ),
    (NX, NY, NZ),

    # +Z
    (NX, NY, NZ + D),
    (NX, NY, NZ),

    # -Z
    (NX, NY, NZ - D),
    (NX, NY, NZ),

    # Return home
    (HOME_X, HOME_Y, HOME_Z),
]

# ==========================================
# WRITE FILE
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