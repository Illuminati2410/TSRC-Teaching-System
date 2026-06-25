import sys
import os
# Force this script to look directly inside your Miniconda library node
sys.path.insert(0, r"C:\Users\Shaurya\miniconda3\Lib\site-packages")
sys.path.insert(
    0,
    r"C:\Users\Shaurya\libsurvive\build-win\Release\tracker_logger\fairino-python-sdk-v2.1.4_robot3.8.4\windows"
)
from fairino import Robot
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
import os

# ==========================================
# FAIRINO IK ANALYSIS — OPTIONAL MODULE
# ==========================================
# Set ENABLE_IK_ANALYSIS = True when a live robot controller
# (or compatible simulator) is reachable at ROBOT_IP.
# When False the script behaves exactly as before; no SDK calls
# are made and fairino_path.txt is written unchanged.
#
# Future scaling hooks live in IKScaleConfig (do not implement yet).
# ==========================================

ENABLE_IK_ANALYSIS = True          # ← flip to False to skip IK entirely
ROBOT_IP           = "192.168.86.128"

# Seed joint angles used for the very first IK call (safe home pose).
# GetInverseKinRef needs a reference; subsequent calls chain automatically.
IK_SEED_JOINTS = [0.0, -60.0, 90.0, -30.0, -90.0, 0.0]

# ── Future scaling architecture (not implemented yet) ──────────────────────────
# When you are ready to add per-axis scaling for trajectory compression / expansion,
# populate IKScaleConfig and pass it into build_trajectory().  All current
# trajectory math stays untouched; the config object is the single extension point.
class IKScaleConfig:
    """
    Placeholder for future per-axis trajectory scaling.

    Fields (not used yet):
        scale_x, scale_y, scale_z  – independent axis multipliers
        auto_scale                 – enable automatic workspace fitting
    """
    scale_x    = None   # not implemented
    scale_y    = None   # not implemented
    scale_z    = None   # not implemented
    auto_scale = False  # not implemented

# ──────────────────────────────────────────────────────────────────────────────


def wrap_angle(angle):
    return (angle + 180) % 360 - 180
# ==========================================
# SETTINGS
# ==========================================

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

INPUT_FILE = os.path.join(BASE_DIR, "data", "tracker_data.csv")
OUTPUT_FILE = os.path.join(BASE_DIR, "simulation", "fairino_path.txt")

SCALE = 0.35

START_X = 260
START_Y = -400
START_Z = 300

START_ROLL = 180
START_PITCH = 0
START_YAW = 90

STEP = 10
DEBUG_START = 175
DEBUG_END = 225
# Maximum allowed jump between saved points (mm)
MAX_JUMP = 150

# Maximum allowed jump between raw tracker samples (mm)
MAX_TRACKER_JUMP = 200
MAX_ORIENTATION_JUMP = 45
MAX_ROBOT_ORIENTATION_STEP = 5.0


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


def normalize_quat(q):
    return q / np.linalg.norm(q)


def keep_quat_same_hemisphere(q, previous_q):
    if previous_q is not None and np.dot(q, previous_q) < 0:
        return -q

    return q
# ==========================================
# LOAD DATA
# ==========================================

df = pd.read_csv(INPUT_FILE)

required_columns = [
    "time",
    "x",
    "y",
    "z",
    "qw",
    "qx",
    "qy",
    "qz",
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
q0 = normalize_quat(q0)
x = df["x"].to_numpy() * 1000
y = df["y"].to_numpy() * 1000
z = df["z"].to_numpy() * 1000
roll = df["unwrapped_roll"].to_numpy()
pitch = df["unwrapped_pitch"].to_numpy()
yaw = df["unwrapped_yaw"].to_numpy()
ROT_SCALE = 0.3
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

def quat_to_rotvec_degrees(q):

    w, x, y, z = q
    q = normalize_quat(np.array([w, x, y, z]))

    # q and -q describe the same orientation. Use the shortest-axis
    # representation before taking the quaternion logarithm.
    if q[0] < 0:
        q = -q

    w, x, y, z = q

    r = R.from_quat([
        x,
        y,
        z,
        w
    ])

    rotvec = np.degrees(r.as_rotvec())

    return rotvec[0], rotvec[1], rotvec[2]

def angle_diff(a, b):

    d = a - b

    while d > 180:
        d -= 360

    while d < -180:
        d += 360

    return d

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
# FAIRINO IK ANALYSIS — INITIALISATION
# ==========================================
# This block runs before the write loop.
# It connects to the robot, fetches real joint soft-limits, and
# prepares all accumulators.  The trajectory write loop is NOT
# changed; analysis hooks are injected inside it below.
# ==========================================

# ── Joint limit thresholds (degrees) ──────────────────────────────────────────
IK_MARGIN_SAFE      = 60.0    # green  — comfortable
IK_MARGIN_WARNING   = 30.0    # amber  — monitor
IK_MARGIN_DANGEROUS = 10.0    # red    — must fix before deployment
# below IK_MARGIN_DANGEROUS → REJECT

# ── SDK connection ─────────────────────────────────────────────────────────────
_robot          = None   # Robot.RPC instance (None when IK disabled / failed)
_joint_limits   = None   # [[lo, hi], …] for J1..J6, fetched from controller

if ENABLE_IK_ANALYSIS:
    try:
        from fairino import Robot as _FairinoRobot
        

        print(f"\nConnecting to FAIRINO controller at {ROBOT_IP} for IK analysis…")
        _robot = _FairinoRobot.RPC(ROBOT_IP)

        # ── Fetch real joint soft limits from the controller ───────────────────
        # GetJointSoftLimitDeg(flag=1)
        #   Returns: (error, [j1min, j1max, j2min, j2max, j3min, j3max,
        #                      j4min, j4max, j5min, j5max, j6min, j6max])
        #   Units: degrees
        _err_lim, _raw_limits = _robot.GetJointSoftLimitDeg(1)

        if _err_lim == 0 and _raw_limits is not None:
            # Reshape flat list into [[lo, hi], …] per joint
            _joint_limits = [
                [_raw_limits[2 * j], _raw_limits[2 * j + 1]]
                for j in range(6)
            ]
            print("Joint soft limits fetched from controller (degrees):")
            for _j, (_lo, _hi) in enumerate(_joint_limits):
                print(f"  J{_j+1}: [{_lo:.1f}, {_hi:.1f}]")
        else:
            print(f"WARNING: GetJointSoftLimitDeg failed (error={_err_lim}). "
                  "IK margin analysis will be skipped.")
            _robot = None   # disable IK analysis gracefully

    except Exception as _e:
        print(f"WARNING: Could not connect to FAIRINO controller ({_e}). "
              "IK analysis disabled; trajectory file will be written unchanged.")
        _robot = None

# ── Per-trajectory accumulators ────────────────────────────────────────────────
_ik_previous_joints = list(IK_SEED_JOINTS)   # seed for first GetInverseKinRef call
_ik_failure_list    = []                       # [(waypoint_index, pose), …]
_ik_all_joints      = []                       # [(waypoint_index, joints), …] — successes only
_unsafe_waypoints = []
# Per-joint running min/max   — indexed 0..5
_ik_joint_min = [ float("inf")] * 6
_ik_joint_max = [float("-inf")] * 6

# Worst-margin tracking
_ik_worst_margin     = float("inf")
_ik_worst_wp_index   = -1
_ik_worst_joint_idx  = -1          # 0-based joint number
_ik_worst_pose       = None
_ik_worst_joints     = None

# Waypoint counter shared with the analysis (mirrors `count` inside the loop)
_ik_waypoint_index   = 0           # incremented once per substep written

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
    prev_q_current = None
    prev_q_rel = None
    orient_roll = 0.0
    orient_pitch = 0.0
    orient_yaw = 0.0
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
        q_current = normalize_quat(q_current)
        q_current = keep_quat_same_hemisphere(
            q_current,
            prev_q_current
        )
        prev_q_current = q_current.copy()

        q_rel = quat_multiply(
            quat_conjugate(q0),
            q_current
        )
        q_rel = normalize_quat(q_rel)
        q_rel = keep_quat_same_hemisphere(
            q_rel,
            prev_q_rel
        )

        if prev_q_rel is None:
            q_delta = q_rel.copy()
        else:
            q_delta = quat_multiply(
                quat_conjugate(prev_q_rel),
                q_rel
            )

        q_delta = normalize_quat(q_delta)
        if q_delta[0] < 0:
            q_delta = -q_delta

        delta_angle = 2 * np.degrees(
            np.arccos(
                np.clip(q_delta[0], -1.0, 1.0)
            )
        )

        if 110 <= count <= 130:
            print(
                f"REAL ROTATION = {delta_angle:.2f} deg"
            )

        step_roll, step_pitch, step_yaw = quat_to_rotvec_degrees(q_delta)

        orient_roll += step_roll
        orient_pitch += step_pitch
        orient_yaw += step_yaw

        prev_q_rel = q_rel.copy()
        #if count < 10:
        #    print(
        #        f"QREL "
        #        f"{q_rel[0]:.4f} "
        #        f"{q_rel[1]:.4f} "
        #        f"{q_rel[2]:.4f} "
        #        f"{q_rel[3]:.4f}"
        #    )

        droll = orient_roll
        dpitch = orient_pitch
        dyaw = orient_yaw
        debug_mode = (
            DEBUG_START <= count <= DEBUG_END
        )
        if count >= 185 and count <= 195:
            print(
                f"ROTVEC "
                f"X={droll:.1f} "
                f"Y={dpitch:.1f} "
                f"Z={dyaw:.1f}"
            )

        ROLL_GAIN = 0.3
        PITCH_GAIN = 0.3
        YAW_GAIN = 0.3

        pr   = START_ROLL  + droll  * ROLL_GAIN
        pp   = START_PITCH + dpitch * PITCH_GAIN
        pyaw = START_YAW   + dyaw   * YAW_GAIN

            #pr   = wrap_angle(pr)
            #pp   = wrap_angle(pp)
            #pyaw = wrap_angle(pyaw)
        start_pr = pr if prev_pr is None else prev_pr
        start_pp = pp if prev_pp is None else prev_pp
        start_pyaw = pyaw if prev_pyaw is None else prev_pyaw

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

            #print(
               # f"STEP R={roll_step:.2f} "
                #f"P={pitch_step:.2f} "
                #f"Y={yaw_step:.2f}"
            #)

        segment_steps = max(
            1,
            int(
                np.ceil(
                    max(
                        roll_step,
                        pitch_step,
                        yaw_step,
                    )
                    / MAX_ROBOT_ORIENTATION_STEP
                )
            )
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

        if debug_mode:
            print(
                f"{count:03d}: "
                f"{px:.3f} "
                f"{py:.3f} "
                f"{pz:.3f}"
            )

        if debug_mode:
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

        if debug_mode:
            print(f"QANGLE={angle:.1f}")

        for substep in range(1, segment_steps + 1):

            alpha = substep / segment_steps

            out_x   = last_px   + (px   - last_px)   * alpha
            out_y   = last_py   + (py   - last_py)   * alpha
            out_z   = last_pz   + (pz   - last_pz)   * alpha
            out_r   = start_pr  + (pr   - start_pr)  * alpha
            out_p   = start_pp  + (pp   - start_pp)  * alpha
            out_yaw = start_pyaw + (pyaw - start_pyaw) * alpha

            # ── FAIRINO IK ANALYSIS HOOK ───────────────────────────────────────
            # Runs only when a live controller is available.
            # Does NOT alter out_x/y/z/r/p/yaw or the file output in any way.
            if _robot is not None and _joint_limits is not None:

                _ik_waypoint_index += 1
                _pose = [out_x, out_y, out_z, out_r, out_p, out_yaw]

                # GetInverseKinRef(type, desc_pos, joint_pos_ref)
                #   type=0  → absolute pose in base frame
                #   joint_pos_ref → previous solution keeps trajectory continuous
                _ik_err, _ik_joints = _robot.GetInverseKinRef(
                    0,
                    _pose,
                    _ik_previous_joints,
                )

                if _ik_err != 0 or _ik_joints is None:
                    # Record failure; do not crash; keep previous seed unchanged
                    _ik_failure_list.append((_ik_waypoint_index, list(_pose)))
                else:
                    # IK succeeded — update seed for next call
                    _ik_previous_joints = list(_ik_joints)
                    _ik_all_joints.append((_ik_waypoint_index, list(_ik_joints)))

                    # ── Update per-joint running min/max ──────────────────────
                    for _j in range(6):
                        if _ik_joints[_j] < _ik_joint_min[_j]:
                            _ik_joint_min[_j] = _ik_joints[_j]
                        if _ik_joints[_j] > _ik_joint_max[_j]:
                            _ik_joint_max[_j] = _ik_joints[_j]

                    # ── Compute margin for every joint at this waypoint ────────
                    # margin = distance to the nearer soft-limit boundary
                    _wp_margins = []
                    for _j in range(6):
                        _lo, _hi = _joint_limits[_j]
                        _margin_lo = _ik_joints[_j] - _lo
                        _margin_hi = _hi - _ik_joints[_j]
                        _wp_margins.append(min(_margin_lo, _margin_hi))

                    _wp_min_margin = min(_wp_margins)
                    if _wp_min_margin < IK_MARGIN_DANGEROUS:

                        _unsafe_waypoints.append(
                            (
                                _ik_waypoint_index,
                                _wp_min_margin,
                                list(_pose)
                            )
                        )

                    # ── Track globally worst margin ────────────────────────────
                    if _wp_min_margin < _ik_worst_margin:
                        _ik_worst_margin    = _wp_min_margin
                        _ik_worst_wp_index  = _ik_waypoint_index
                        _ik_worst_joint_idx = _wp_margins.index(_wp_min_margin)
                        _ik_worst_pose      = list(_pose)
                        _ik_worst_joints    = list(_ik_joints)
            # ── END IK ANALYSIS HOOK ──────────────────────────────────────────

            f.write(
                f"{out_x:.3f} "
                f"{out_y:.3f} "
                f"{out_z:.3f} "
                f"{out_r:.3f} "
                f"{out_p:.3f} "
                f"{out_yaw:.3f}\n"
            )

        last_px = px
        last_py = py
        last_pz = pz

        count += 1

# ==========================================
# FAIRINO IK ANALYSIS — END-OF-RUN REPORT
# ==========================================
# Printed after fairino_path.txt is fully written and closed.
# The file on disk is identical regardless of whether this block runs.
# ==========================================

if _robot is not None and _joint_limits is not None:

    _total_waypoints = _ik_waypoint_index
    _ik_failure_count = len(_ik_failure_list)
    _ik_success_count = len(_ik_all_joints)

    # ── Determine overall classification ──────────────────────────────────────
    if _ik_worst_margin == float("inf"):
        # No successful IK at all
        _classification = "NO DATA"
    elif _ik_worst_margin > IK_MARGIN_SAFE:
        _classification = "SAFE"
    elif _ik_worst_margin > IK_MARGIN_WARNING:
        _classification = "WARNING"
    elif _ik_worst_margin > IK_MARGIN_DANGEROUS:
        _classification = "DANGEROUS"
    else:
        _classification = "REJECT"

    print("\n" + "=" * 60)
    print("TRAJECTORY JOINT ANALYSIS")
    print("=" * 60)

    print(f"\nTotal Waypoints  : {_total_waypoints}")
    print(f"IK Successes     : {_ik_success_count}")
    print(f"IK Failures      : {_ik_failure_count}")

    if _ik_failure_count > 0:
        print("\nFailed Waypoints (index, pose):")
        for _wp_i, _wp_pose in _ik_failure_list:
            _xf, _yf, _zf, _rf, _pf, _yf2 = _wp_pose
            print(
                f"  WP {_wp_i:05d}: "
                f"X={_xf:.2f} Y={_yf:.2f} Z={_zf:.2f} "
                f"R={_rf:.2f} P={_pf:.2f} Yaw={_yf2:.2f}"
            )

    print("\nJoint Ranges (degrees):")
    print(f"  {'Joint':<6} {'Min':>10} {'Max':>10}  {'Soft Lo':>10} {'Soft Hi':>10}")
    for _j in range(6):
        _lo, _hi = _joint_limits[_j]
        _jmin = _ik_joint_min[_j] if _ik_joint_min[_j] != float("inf") else float("nan")
        _jmax = _ik_joint_max[_j] if _ik_joint_max[_j] != float("-inf") else float("nan")
        print(
            f"  J{_j+1:<5} "
            f"{_jmin:>10.2f} "
            f"{_jmax:>10.2f}  "
            f"{_lo:>10.2f} "
            f"{_hi:>10.2f}"
        )
    print("\nJoint Usage:")

    for _j in range(6):

        _lo, _hi = _joint_limits[_j]

        _jmin = _ik_joint_min[_j]
        _jmax = _ik_joint_max[_j]

        if (
            _jmin == float("inf")
            or _jmax == float("-inf")
        ):
            continue

        _used_range = _jmax - _jmin
        _full_range = _hi - _lo

        _usage_pct = 100.0 * _used_range / _full_range

        print(
            f"  J{_j+1}: {_usage_pct:.1f}%"
        )

    if _ik_worst_joints is not None:
        print(f"\nWorst Margin:")
        print(f"  Joint         : J{_ik_worst_joint_idx + 1}")
        print(f"  Margin        : {_ik_worst_margin:.2f} deg")
        print(f"  Waypoint Index: {_ik_worst_wp_index}")
        _wx, _wy, _wz, _wr, _wp2, _wy2 = _ik_worst_pose
        print(
            f"  Cartesian Pose: "
            f"X={_wx:.3f} Y={_wy:.3f} Z={_wz:.3f} "
            f"Roll={_wr:.3f} Pitch={_wp2:.3f} Yaw={_wy2:.3f}"
        )
        _j_vals = "  ".join(f"J{_j+1}={_ik_worst_joints[_j]:.2f}" for _j in range(6))
        print(f"  Joint Values  : {_j_vals}")
    else:
        print("\nWorst Margin: no successful IK solutions to report.")

    print(f"\nOverall Classification: {_classification}")
    print(
        f"\nUnsafe Waypoints: "
        f"{len(_unsafe_waypoints)}"
    )

    if len(_unsafe_waypoints) > 0:

        print("\nFirst Unsafe Waypoints:")

        for _wp, _margin, _pose in _unsafe_waypoints[:10]:

            print(
                f"  WP {_wp} "
                f"Margin={_margin:.2f}"
            )
    print(
        "  SAFE (>60°)  WARNING (30–60°)  DANGEROUS (10–30°)  REJECT (<10°)"
    )

    print("\n" + "=" * 60)

elif ENABLE_IK_ANALYSIS:
    # ENABLE_IK_ANALYSIS was True but connection failed at startup
    print("\n[IK ANALYSIS] Skipped — controller not reachable at startup.")
else:
    print("\n[IK ANALYSIS] Disabled (ENABLE_IK_ANALYSIS = False).")