import pandas as pd
import numpy as np
import os
def wrap_angle(angle):
    return (angle + 180) % 360 - 180
# ==========================================
# SETTINGS
# ==========================================

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

INPUT_FILE = os.path.join(BASE_DIR, "data", "tracker_data.csv")
OUTPUT_FILE = os.path.join(BASE_DIR, "simulation", "fairino_path.txt")

SCALE = 0.2

START_X = 260
START_Y = -400
START_Z = 300

START_ROLL = 180
START_PITCH = 0
START_YAW = 90

STEP = 10

# Maximum allowed jump between saved points (mm)
MAX_JUMP = 150

# Maximum allowed jump between raw tracker samples (mm)
MAX_TRACKER_JUMP = 200
MAX_ORIENTATION_JUMP = 45
# ==========================================
# LOAD DATA
# ==========================================

df = pd.read_csv(INPUT_FILE)

required_columns = [
    "time",
    "x",
    "y",
    "z",
    "unwrapped_roll",
    "unwrapped_pitch",
    "unwrapped_yaw",
]
missing_columns = [col for col in required_columns if col not in df.columns]

if missing_columns:
    raise ValueError(f"Missing columns in tracker CSV: {missing_columns}")

if df.empty:
    raise ValueError("tracker_data.csv has no samples")

x = df["x"].to_numpy() * 1000
y = df["y"].to_numpy() * 1000
z = df["z"].to_numpy() * 1000
roll = df["unwrapped_roll"].to_numpy()
pitch = df["unwrapped_pitch"].to_numpy()
yaw = df["unwrapped_yaw"].to_numpy()
ROT_SCALE = 0.5
PITCH_SCALE = 0.3

roll0 = roll[0]
pitch0 = pitch[0]
yaw0 = yaw[0]
# ==========================================
# FIND BAD TRACKER JUMPS
# ==========================================

bad_rows = []

for i in range(1, len(x)):

    dx = abs(x[i] - x[i - 1])
    dy = abs(y[i] - y[i - 1])
    dz = abs(z[i] - z[i - 1])

    if (
        dx > MAX_TRACKER_JUMP
        or dy > MAX_TRACKER_JUMP
        or dz > MAX_TRACKER_JUMP
    ):

        bad_rows.append(i)

        print(
            f"BAD ROW {i} "
            f"dx={dx:.1f} "
            f"dy={dy:.1f} "
            f"dz={dz:.1f}"
        )

print("\nBad rows found:", bad_rows)
# ==========================================
# TRAJECTORY CENTER
# ==========================================

tracker_x0 = x[0]
tracker_y0 = y[0]
tracker_z0 = z[0]

print("\nTracker Reference Pose")
print(f"X0 = {tracker_x0:.1f}")
print(f"Y0 = {tracker_y0:.1f}")
print(f"Z0 = {tracker_z0:.1f}")

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
# CHECK RAW TRACKER JUMPS
# ==========================================

print("\nChecking raw tracker data for large jumps...")

tracker_jump_count = 0

for i in range(1, len(x)):

    dx = abs(x[i] - x[i - 1])
    dy = abs(y[i] - y[i - 1])
    dz = abs(z[i] - z[i - 1])

    if dx > MAX_TRACKER_JUMP or dy > MAX_TRACKER_JUMP or dz > MAX_TRACKER_JUMP:

        tracker_jump_count += 1

        print(f"\nRAW TRACKER JUMP #{tracker_jump_count}")
        print(f"CSV rows: {i} -> {i + 1}")

        print(
            f"Prev: "
            f"{x[i-1]:.1f}, "
            f"{y[i-1]:.1f}, "
            f"{z[i-1]:.1f}"
        )

        print(
            f"Curr: "
            f"{x[i]:.1f}, "
            f"{y[i]:.1f}, "
            f"{z[i]:.1f}"
        )

        print(
            f"Delta: "
            f"{dx:.1f}, "
            f"{dy:.1f}, "
            f"{dz:.1f}"
        )

print(f"\nTotal raw tracker jumps found = {tracker_jump_count}")

# ==========================================
# TRANSFORM TO ROBOT SPACE
# ==========================================

dx = x - tracker_x0
dy = y - tracker_y0
dz = z - tracker_z0

robot_x = START_X + dx * SCALE
robot_y = START_Y + dy * SCALE
robot_z = START_Z + dz * SCALE

robot_x = np.clip(robot_x, 150, 550)
robot_y = np.clip(robot_y, -550, -350)
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

os.makedirs(os.path.dirname(OUTPUT_FILE), exist_ok=True)

with open(OUTPUT_FILE, "w") as f:

    # ----------------------------------
    # SAFE START POSE
    # ----------------------------------

    f.write(
        f"{START_X:.3f} "
        f"{START_Y:.3f} "
        f"{START_Z:.3f} "
        f"{START_ROLL:.3f} "
        f"{START_PITCH:.3f} "
        f"{START_YAW:.3f}\n"
    )

    print(
        "\nSAFE START POSE:",
        f"{START_X:.1f}",
        f"{START_Y:.1f}",
        f"{START_Z:.1f}",
        f"{START_ROLL:.1f}",
        f"{START_PITCH:.1f}",
        f"{START_YAW:.1f}",
    )

    count = 1


    last_px = START_X
    last_py = START_Y
    last_pz = START_Z

    for i in range(STEP, len(robot_x), STEP):

        SKIP_RADIUS = 20

        skip_point = False

        for bad_row in bad_rows:

            if abs(i - bad_row) <= SKIP_RADIUS:
                skip_point = True
                break

        if skip_point:

            skipped += 1

            print(
                f"SKIPPED BAD TRACKER REGION row={i}"
            )

            continue

        px = robot_x[i]
        py = robot_y[i]
        pz = robot_z[i]

        pr = START_ROLL

        roll_offset = roll[i] - roll0
        pp = START_PITCH + roll_offset * PITCH_SCALE

        yaw_offset = yaw[i] - yaw0
        pyaw = START_YAW + yaw_offset * ROT_SCALE

        pp = wrap_angle(pp)
        pyaw = wrap_angle(pyaw)

        dx = abs(px - last_px)
        dy = abs(py - last_py)
        dz = abs(pz - last_pz)

        if (
            dx > MAX_JUMP
            or dy > MAX_JUMP
            or dz > MAX_JUMP
        ):

            skipped += 1

            print(
                f"SKIPPED JUMP "
                f"dx={dx:.1f} "
                f"dy={dy:.1f} "
                f"dz={dz:.1f}"
            )

            continue

        print(
            f"{count:03d}: "
            f"{px:.3f} "
            f"{py:.3f} "
            f"{pz:.3f}"
        )

        print(
            f"RollOffset={roll_offset:.1f} "
            f"RobotPitch={pp:.1f} "
            f"YawOffset={yaw_offset:.1f} "
            f"RobotYaw={pyaw:.1f}"
        )

        f.write(
            f"{px:.3f} "
            f"{py:.3f} "
            f"{pz:.3f} "
            f"{pr:.3f} "
            f"{pp:.3f} "
            f"{pyaw:.3f}\n"
        )

        last_px = px
        last_py = py
        last_pz = pz

        count += 1

print("len(robot_x) =", len(robot_x))
print("STEP =", STEP)
print("Expected points =", len(range(0, len(robot_x), STEP)))

print("\nPoints written =", count)
print("Points skipped =", skipped)

print(f"\nSaved: {OUTPUT_FILE}")

print("\nFirst generated pose")

print(
    f"{robot_x[0]:.3f} "
    f"{robot_y[0]:.3f} "
    f"{robot_z[0]:.3f} "
    f"{roll[0]:.3f} "
    f"{pitch[0]:.3f} "
    f"{yaw[0]:.3f}"
)

print(
    "\nFile size:",
    os.path.getsize(OUTPUT_FILE),
    "bytes"
)

print("\nRobot trajectory span")

print(
    f"X span = {robot_x.max()-robot_x.min():.1f}"
)

print(
    f"Y span = {robot_y.max()-robot_y.min():.1f}"
)

print(
    f"Z span = {robot_z.max()-robot_z.min():.1f}"
)

print("Roll :", roll.min(), roll.max())
print("Pitch:", pitch.min(), pitch.max())
print("Yaw  :", yaw.min(), yaw.max())

print("\nReference Offsets")
print(f"tracker_x0 = {tracker_x0:.1f}")
print(f"tracker_y0 = {tracker_y0:.1f}")
print(f"tracker_z0 = {tracker_z0:.1f}")

print(f"tracker_x_median = {np.median(x):.1f}")
print(f"tracker_y_median = {np.median(y):.1f}")
print(f"tracker_z_median = {np.median(z):.1f}")