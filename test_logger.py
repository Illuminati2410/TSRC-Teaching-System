"""Lightweight tracker data logger using unified TrackerReader."""

import csv
import math
import os
import sys
import time

from tracker_core import TrackerReader


SURVIVE_EXE = r"C:\Users\Shaurya\libsurvive\build-win\Release\survive-cli.exe"
TARGET_DEVICE = "WM0"

base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
csv_file = os.path.join(base_dir, "data", "tracker_data.csv")


def quat_to_rpy(qw, qx, qy, qz):
    """Convert quaternion to Euler angles (roll, pitch, yaw in degrees)."""
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
    """Unwrap angle discontinuities."""
    if previous is None:
        return current

    while current - previous > 180.0:
        current -= 360.0

    while current - previous < -180.0:
        current += 360.0

    return current


def main():
    """Main logger loop."""
    print("=" * 64)
    print("TSRC Tracker Logger")
    print("=" * 64)

    # Create reader
    reader = TrackerReader(SURVIVE_EXE, TARGET_DEVICE)
    
    if not reader.start():
        print("ERROR: Failed to start tracker reader")
        return 1

    # Prepare CSV
    os.makedirs(os.path.dirname(csv_file), exist_ok=True)

    # State for angle unwrapping
    last_roll = None
    last_pitch = None
    last_yaw = None
    last_seq = -1
    
    # Tracker reference (will be populated after stabilization)
    tracker_x0 = None
    tracker_y0 = None
    tracker_z0 = None
    reference_captured = False

    # Open CSV for writing
    with open(csv_file, "w", newline="") as f:
        writer = csv.writer(f)

        # Write header
        writer.writerow([
            "time",
            "x",
            "y",
            "z",
            "tracker_x0",
            "tracker_y0",
            "tracker_z0",
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
        f.flush()

        try:
            print("\nLogging tracker data. Press Ctrl+C to stop.\n")

            while True:
                pose = reader.get_latest_pose()
                
                if pose is None:
                    # Avoid busy waiting: sleep briefly when no new pose
                    time.sleep(0.005)
                    continue
                
                # Skip duplicate samples
                if pose.seq == last_seq:
                    # Avoid busy waiting: sleep briefly when no new pose
                    time.sleep(0.005)
                    continue
                
                last_seq = pose.seq
                
                # Capture tracker neutral reference on first valid pose
                if not reference_captured:
                    tracker_x0, tracker_y0, tracker_z0 = reader.get_tracker_neutral_reference()
                    reference_captured = True

                # Compute Euler angles
                raw_roll, raw_pitch, raw_yaw = quat_to_rpy(
                    pose.qw, pose.qx, pose.qy, pose.qz
                )

                # Unwrap angles
                unwrapped_roll = unwrap_angle(last_roll, raw_roll)
                unwrapped_pitch = unwrap_angle(last_pitch, raw_pitch)
                unwrapped_yaw = unwrap_angle(last_yaw, raw_yaw)

                last_roll = unwrapped_roll
                last_pitch = unwrapped_pitch
                last_yaw = unwrapped_yaw

                # Print to console
                x_mm = pose.x * 1000
                y_mm = pose.y * 1000
                z_mm = pose.z * 1000
                
                print(
                    f"t={pose.timestamp:.2f} "
                    f"X={x_mm:.1f} mm "
                    f"Y={y_mm:.1f} mm "
                    f"Z={z_mm:.1f} mm "
                    f"R={raw_roll:.1f}° "
                    f"P={raw_pitch:.1f}° "
                    f"Y={raw_yaw:.1f}°"
                )

                # Write to CSV with actual tracker reference values
                writer.writerow([
                    pose.timestamp,
                    pose.x,
                    pose.y,
                    pose.z,
                    tracker_x0 if tracker_x0 is not None else 0.0,
                    tracker_y0 if tracker_y0 is not None else 0.0,
                    tracker_z0 if tracker_z0 is not None else 0.0,
                    pose.qx,
                    pose.qy,
                    pose.qz,
                    pose.qw,
                    raw_roll,
                    raw_pitch,
                    raw_yaw,
                    unwrapped_roll,
                    unwrapped_pitch,
                    unwrapped_yaw,
                ])
                f.flush()

        except KeyboardInterrupt:
            print("\n\nLogging stopped by user.")
        finally:
            reader.stop()
            print(f"Data saved to: {csv_file}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
