import subprocess
import math
import csv
import os

base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
csv_file = os.path.join(base_dir, "data", "tracker_data.csv")

MAX_JUMP_MM = 200
RESYNC_AFTER_STABLE_REJECTS = 5

def quat_to_rpy(qw, qx, qy, qz):
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw)
    )


def unwrap_angle(previous, current):
    if previous is None:
        return current

    while current - previous > 180.0:
        current -= 360.0

    while current - previous < -180.0:
        current += 360.0

    return current


last_x = None
last_y = None
last_z = None
reject_x = None
reject_y = None
reject_z = None
stable_rejects = 0
last_roll = None
last_pitch = None
last_yaw = None

# ── Orientation reference (set on first accepted sample in recording loop) ────
q_ref_w = None
q_ref_x = None
q_ref_y = None
q_ref_z = None

os.makedirs(os.path.dirname(csv_file), exist_ok=True)

with open(csv_file, "w", newline="") as f:
    writer = csv.writer(f)

    writer.writerow([
        "time",
        "x",
        "y",
        "z",
        "tracker_x0",
        "tracker_y0",
        "tracker_z0",
        # survive-cli --record-stdout emits:  tx ty tz  qx qy qz qw  (scalar LAST)
        # parts[6]=qx  parts[7]=qy  parts[8]=qz  parts[9]=qw
        # Labels corrected to match actual field positions.
        "qx",
        "qy",
        "qz",
        "qw",
        "raw_roll",
        "raw_pitch",
        "raw_yaw",
        "unwrapped_roll",
        "unwrapped_pitch",
        "unwrapped_yaw", 
    ])

cmd = [
    r"C:\Users\Shaurya\libsurvive\build-win\Release\survive-cli.exe",
    "--record-stdout"
]

print("Recording... Press Ctrl+C to stop")

proc = subprocess.Popen(
    cmd,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    bufsize=1
)
print("\nTracker stabilization...")
print("Please hold tracker still...")

stabilization_samples = []

while len(stabilization_samples) < 100:

    line = proc.stdout.readline()

    if "POSE" not in line:
        continue

    parts = line.split()

    try:

        t = float(parts[0])

        if t < 3.0:
            continue

        x = float(parts[3]) * 1000
        y = float(parts[4]) * 1000
        z = float(parts[5]) * 1000

        stabilization_samples.append((x, y, z))

    except:
        continue

tracker_x0 = sum(p[0] for p in stabilization_samples) / len(stabilization_samples)
tracker_y0 = sum(p[1] for p in stabilization_samples) / len(stabilization_samples)
tracker_z0 = sum(p[2] for p in stabilization_samples) / len(stabilization_samples)

print("\nTracker Neutral Reference")
print(f"X0 = {tracker_x0:.3f}")
print(f"Y0 = {tracker_y0:.3f}")
print(f"Z0 = {tracker_z0:.3f}")

print("\nStarting actual recording...\n")
for line in proc.stdout:

    if "POSE" not in line:
        continue

    print(line.strip())   # ADD THIS

    parts = line.split()

    try:
        t = float(parts[0])
        if t < 3.0:
            continue

        x = float(parts[3])
        y = float(parts[4])
        z = float(parts[5])
        # Reject impossible tracker solutions

        # Reject obviously impossible positions

        if abs(x) > 5 or abs(y) > 5 or abs(z) > 5:
            print(
                f"BAD TRACK: "
                f"{x:.3f} {y:.3f} {z:.3f}"
            )
            continue

        # survive-cli --record-stdout field order (verified against libsurvive source):
        #   parts[0]=time  parts[1]="POSE"  parts[2]=tag
        #   parts[3]=tx    parts[4]=ty      parts[5]=tz
        #   parts[6]=qx    parts[7]=qy      parts[8]=qz    parts[9]=qw   (scalar LAST)
        #
        # Previous logger had these labelled qw/qx/qy/qz (shifted by one).
        # Corrected below.  Internal convention kept as [w, x, y, z] for all
        # downstream math (same as before) — only the parse assignment changed.
        qx = float(parts[6])   # was incorrectly labelled qw
        qy = float(parts[7])   # was incorrectly labelled qx
        qz = float(parts[8])   # was incorrectly labelled qy
        qw = float(parts[9])   # was incorrectly labelled qz

        q_norm_val = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)

        print(
            f"RAW QUAT: qx={parts[6]} qy={parts[7]} qz={parts[8]} qw={parts[9]}"
        )
        print(f"Q NORM = {q_norm_val:.4f}")

        # Normalise
        if q_norm_val < 1e-6:
            print("DEGENERATE QUATERNION — skipping")
            continue
        qw /= q_norm_val
        qx /= q_norm_val
        qy /= q_norm_val
        qz /= q_norm_val

        # ── Orientation reference: capture on first accepted sample ──────────
        # q_ref is the tracker orientation at the start of recording.
        # All subsequent orientations are expressed as rotations RELATIVE to
        # this reference, so the robot returns to its neutral pose when the
        # tracker returns to its start position.
        if q_ref_w is None:
            q_ref_w = qw
            q_ref_x = qx
            q_ref_y = qy
            q_ref_z = qz
            print(
                f"ORIENTATION REFERENCE SET: "
                f"qw={q_ref_w:.6f} qx={q_ref_x:.6f} "
                f"qy={q_ref_y:.6f} qz={q_ref_z:.6f}"
            )

        # ── Relative quaternion: q_rel = conj(q_ref) * q_current ─────────────
        # conj(q_ref) = [q_ref_w, -q_ref_x, -q_ref_y, -q_ref_z]
        # Product using Hamilton product (internal [w,x,y,z] convention):
        aw, ax, ay, az = q_ref_w, -q_ref_x, -q_ref_y, -q_ref_z  # conj(q_ref)
        bw, bx, by, bz = qw, qx, qy, qz
        rel_w = aw*bw - ax*bx - ay*by - az*bz
        rel_x = aw*bx + ax*bw + ay*bz - az*by
        rel_y = aw*by - ax*bz + ay*bw + az*bx
        rel_z = aw*bz + ax*by - ay*bx + az*bw
        # Normalise
        rel_norm = math.sqrt(rel_w**2 + rel_x**2 + rel_y**2 + rel_z**2)
        if rel_norm > 1e-6:
            rel_w /= rel_norm; rel_x /= rel_norm
            rel_y /= rel_norm; rel_z /= rel_norm
        # Rotation angle magnitude of q_rel
        rel_angle_deg = 2.0 * math.degrees(math.acos(max(-1.0, min(1.0, rel_w))))

        print(
            f"NORMALIZED Q: qw={qw:.6f} qx={qx:.6f} qy={qy:.6f} qz={qz:.6f}"
        )
        print(
            f"DELTA Q: qw={rel_w:.6f} qx={rel_x:.6f} "
            f"qy={rel_y:.6f} qz={rel_z:.6f}"
        )
        print(f"ANGLE DEG: {rel_angle_deg:.2f}")

        raw_roll, raw_pitch, raw_yaw = quat_to_rpy(
            qw, qx, qy, qz
        )
        # Reject impossible jumps

        

        x_mm = x * 1000
        y_mm = y * 1000
        z_mm = z * 1000
        if last_x is not None:

            dx = abs(x_mm - last_x)
            dy = abs(y_mm - last_y)
            dz = abs(z_mm - last_z)

            print(
                f"JUMP CHECK "
                f"current=({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) "
                f"previous=({last_x:.1f}, {last_y:.1f}, {last_z:.1f}) "
                f"delta=({dx:.1f}, {dy:.1f}, {dz:.1f})"
            )

            if dx > MAX_JUMP_MM or dy > MAX_JUMP_MM or dz > MAX_JUMP_MM:

                if reject_x is None:
                    stable_rejects = 1
                else:
                    reject_dx = abs(x_mm - reject_x)
                    reject_dy = abs(y_mm - reject_y)
                    reject_dz = abs(z_mm - reject_z)

                    if (
                        reject_dx <= MAX_JUMP_MM
                        and reject_dy <= MAX_JUMP_MM
                        and reject_dz <= MAX_JUMP_MM
                    ):
                        stable_rejects += 1
                    else:
                        stable_rejects = 1

                reject_x = x_mm
                reject_y = y_mm
                reject_z = z_mm

                print(
                    f"JUMP REJECTED "
                    f"dx={dx:.1f} "
                    f"dy={dy:.1f} "
                    f"dz={dz:.1f}"
                )

                if stable_rejects < RESYNC_AFTER_STABLE_REJECTS:
                    continue

                print(
                    f"RESYNC ACCEPTED after "
                    f"{stable_rejects} stable rejected samples"
                )
        else:
            print(
                f"JUMP CHECK "
                f"current=({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) "
                f"previous=(None, None, None) "
                f"delta=(0.0, 0.0, 0.0)"
            )

        last_x = x_mm
        last_y = y_mm
        last_z = z_mm
        reject_x = None
        reject_y = None
        reject_z = None
        stable_rejects = 0

        unwrapped_roll = unwrap_angle(
            last_roll,
            raw_roll
        )

        unwrapped_pitch = unwrap_angle(
            last_pitch,
            raw_pitch
        )

        unwrapped_yaw = unwrap_angle(
            last_yaw,
            raw_yaw
        )

        last_roll = unwrapped_roll
        last_pitch = unwrapped_pitch
        last_yaw = unwrapped_yaw

        roll = raw_roll
        pitch = raw_pitch
        yaw = raw_yaw

        print(
            f"t={t:.2f} "
            f"X={x_mm:.1f} mm "
            f"Y={y_mm:.1f} mm "
            f"Z={z_mm:.1f} mm "
            f"R={roll:.1f}° "
            f"P={pitch:.1f}° "
            f"Y={yaw:.1f}°"
        )

        print(
            f"Q = "
            f"{qw:.6f} "
            f"{qx:.6f} "
            f"{qy:.6f} "
            f"{qz:.6f}"
        )

        print(
            f"UNWRAPPED RPY "
            f"R={unwrapped_roll:.1f} "
            f"P={unwrapped_pitch:.1f} "
            f"Y={unwrapped_yaw:.1f}"
        )

        with open(csv_file, "a", newline="") as f:
            writer = csv.writer(f)

            writer.writerow([
                t,
                x,
                y,
                z,
                tracker_x0 / 1000.0,
                tracker_y0 / 1000.0,
                tracker_z0 / 1000.0,
                # Corrected order: qx qy qz qw (scalar-last, matching header)
                qx,
                qy,
                qz,
                qw,
                raw_roll,
                raw_pitch,
                raw_yaw,
                unwrapped_roll,
                unwrapped_pitch,
                unwrapped_yaw,
            ])

            f.flush()

    except Exception as e:
        print("Parse Error:", e)