import subprocess
import math
import csv
import os

with open("tracker_data.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(
        ["time","x","y","z","roll","pitch","yaw"]
    )

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

csv_file = "tracker_data.csv"
last_x = None
last_y = None
last_z = None

if not os.path.exists(csv_file):
    with open(csv_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["time", "x", "y", "z", "roll", "pitch", "yaw"]
        )

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

        qw = float(parts[6])
        qx = float(parts[7])
        qy = float(parts[8])
        qz = float(parts[9])

        roll, pitch, yaw = quat_to_rpy(
            qw, qx, qy, qz
        )
        # Reject impossible jumps

        

        x_mm = x * 1000
        y_mm = y * 1000
        z_mm = z * 1000

        print(
            f"t={t:.2f} "
            f"X={x_mm:.1f} mm "
            f"Y={y_mm:.1f} mm "
            f"Z={z_mm:.1f} mm "
            f"R={roll:.1f}° "
            f"P={pitch:.1f}° "
            f"Y={yaw:.1f}°"
        )

        with open(csv_file, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [t, x, y, z, roll, pitch, yaw]
            )

    except Exception as e:
        print("Parse Error:", e)

