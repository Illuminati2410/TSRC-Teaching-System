import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
import os

def unwrap_angle(previous, current):

    if previous is None:
        return current

    while current - previous > 180:
        current -= 360

    while current - previous < -180:
        current += 360

    return current

def nearest_euler_solution(
    roll,
    pitch,
    yaw,
    prev_roll,
    prev_pitch,
    prev_yaw
):

    candidates = [

        (roll, pitch, yaw),

        (roll + 360, pitch, yaw),
        (roll - 360, pitch, yaw),

        (roll + 180, 180 - pitch, yaw + 180),
        (roll - 180, 180 - pitch, yaw - 180),

        (roll + 180, 180 - pitch, yaw - 180),
        (roll - 180, 180 - pitch, yaw + 180)
    ]

    best = candidates[0]
    best_cost = 1e9

    for r, p, y in candidates:

        cost = (
            abs(r - prev_roll)
            + abs(p - prev_pitch)
            + abs(y - prev_yaw)
        )

        if cost < best_cost:
            best_cost = cost
            best = (r, p, y)

    return best

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


def quat_conjugate(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])

def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b

    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw
    ])
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
qw = df["qw"].to_numpy()
qx = df["qx"].to_numpy()
qy = df["qy"].to_numpy()
qz = df["qz"].to_numpy()
q0 = np.array([
    qw[0],
    qx[0],
    qy[0],
    qz[0]
])
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


def quat_to_rpy(q):

    w, x, y, z = q

    sinr_cosp = 2 * (w*x + y*z)
    cosr_cosp = 1 - 2 * (x*x + y*y)
    roll = np.degrees(np.arctan2(sinr_cosp, cosr_cosp))

    sinp = 2 * (w*y - z*x)

    if abs(sinp) >= 1:
        pitch = np.degrees(np.sign(sinp) * np.pi/2)
    else:
        pitch = np.degrees(np.arcsin(sinp))

    siny_cosp = 2 * (w*z + x*y)
    cosy_cosp = 1 - 2 * (y*y + z*z)
    yaw = np.degrees(np.arctan2(siny_cosp, cosy_cosp))

    return roll, pitch, yaw

def quat_to_rpy_scipy(q):

    w, x, y, z = q

    r = R.from_quat([
        x,
        y,
        z,
        w
    ])

    roll, pitch, yaw = r.as_euler(
        'xyz',
        degrees=True
    )

    return roll, pitch, yaw
# ==========================================
# FIND BAD TRACKER JUMPS
# ==========================================

bad_rows = []

for i in range(1, len(x)):

    dx = abs(x[i] - x[i - 1])
    dy = abs(y[i] - y[i - 1])
    dz = abs(z[i] - z[i - 1])
    #droll = abs(roll[i] - roll[i - 1])
    #dpitch = abs(pitch[i] - pitch[i - 1])
    #dyaw = abs(yaw[i] - yaw[i - 1])

    if (
        dx > MAX_TRACKER_JUMP
        or dy > MAX_TRACKER_JUMP
        or dz > MAX_TRACKER_JUMP
        #or droll > 20
        #or dpitch > 20
        #or dyaw > 20
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

all_roll = []
all_pitch = []
all_yaw = []

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
max_roll_step = 0
max_pitch_step = 0
max_yaw_step = 0
max_roll_index = -1
max_pitch_index = -1
max_yaw_index = -1
prev_pr = None
prev_pp = None
prev_pyaw = None
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
    prev_droll = None
    prev_dpitch = None
    prev_dyaw = None
    for i in range(STEP, len(robot_x), STEP):

        SKIP_RADIUS = 0

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

        q_current = np.array([
            qw[i],
            qx[i],
            qy[i],
            qz[i]
        ])

        q_rel = quat_multiply(
            quat_conjugate(q0),
            q_current
        )
        q_rel = q_rel / np.linalg.norm(q_rel)
        if count < 10:
            print(
                f"QREL "
                f"{q_rel[0]:.4f} "
                f"{q_rel[1]:.4f} "
                f"{q_rel[2]:.4f} "
                f"{q_rel[3]:.4f}"
            )

        droll, dpitch, dyaw = quat_to_rpy_scipy(q_rel)

        if prev_droll is not None:

            droll, dpitch, dyaw = nearest_euler_solution(
                droll,
                dpitch,
                dyaw,
                prev_droll,
                prev_dpitch,
                prev_dyaw
            )

            if abs(droll - prev_droll) > 60:
                print("ROLL FLIP")
                print(prev_droll, "->", droll)

            if abs(dyaw - prev_dyaw) > 60:
                print("YAW FLIP")
                print(prev_dyaw, "->", dyaw)

        MAX_PITCH = 75

        if dpitch > MAX_PITCH:
            dpitch = MAX_PITCH

        if dpitch < -MAX_PITCH:
            dpitch = -MAX_PITCH

        if abs(droll) > 180:
            print("ROLL WARNING", droll)

        if abs(dpitch) > 180:
            print("PITCH WARNING", dpitch)

        if abs(dyaw) > 180:
            print("YAW WARNING", dyaw)

        prev_droll = droll
        prev_dpitch = dpitch
        prev_dyaw = dyaw

        ROLL_GAIN = 0.3
        PITCH_GAIN = 0.3
        YAW_GAIN = 0.3

        pr   = START_ROLL  + droll  * ROLL_GAIN
        pp   = START_PITCH + dpitch * PITCH_GAIN
        pyaw = START_YAW   + dyaw   * YAW_GAIN

            #pr   = wrap_angle(pr)
            #pp   = wrap_angle(pp)
            #pyaw = wrap_angle(pyaw)
        if prev_pr is None:

            roll_step = 0
            pitch_step = 0
            yaw_step = 0

        else:

            roll_step = abs(pr - prev_pr)
            pitch_step = abs(pp - prev_pp)
            yaw_step = abs(pyaw - prev_pyaw)

            if roll_step > max_roll_step:
                max_roll_step = roll_step
                max_roll_index = count

            if pitch_step > max_pitch_step:
                max_pitch_step = pitch_step
                max_pitch_index = count

            if yaw_step > max_yaw_step:
                max_yaw_step = yaw_step
                max_yaw_index = count

                print(
            f"STEP R={roll_step:.2f} "
            f"P={pitch_step:.2f} "
            f"Y={yaw_step:.2f}"
        )

        prev_pr = pr
        prev_pp = pp
        prev_pyaw = pyaw

        all_roll.append(pr)
        all_pitch.append(pp)
        all_yaw.append(pyaw)

        dx = abs(px - last_px)
        dy = abs(py - last_py)
        dz = abs(pz - last_pz)

        print(
            f"{count:03d}: "
            f"{px:.3f} "
            f"{py:.3f} "
            f"{pz:.3f}"
        )

        print(
            f"dRoll={droll:.1f} "
            f"dPitch={dpitch:.1f} "
            f"dYaw={dyaw:.1f}"
        )

        angle = 2 * np.degrees(
            np.arccos(
                np.clip(q_rel[0], -1.0, 1.0)
            )
        )

        print(f"QANGLE={angle:.1f}")

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

