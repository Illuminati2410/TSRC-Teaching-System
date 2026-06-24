# trajectory.py
"""
Trajectory module handling CSV reading/writing, trajectory smoothing,
and joint speed validation.
"""

import os
import csv
import config

def smooth_trajectory_positions(raw_points, window_size=5):
    """
    Applies a moving average filter to the X, Y, Z coordinates to reduce jitter.
    raw_points: list of dicts with 'position' [x, y, z] and 'quaternion' [w, x, y, z]
    """
    if window_size <= 1 or len(raw_points) < window_size:
        return raw_points

    smoothed_points = []
    n = len(raw_points)

    for i in range(n):
        # Determine the sliding window bounds
        start_idx = max(0, i - window_size // 2)
        end_idx = min(n, i + window_size // 2 + 1)
        window = raw_points[start_idx:end_idx]
        count = len(window)

        # Average positions
        avg_pos = [
            sum(pt["position"][0] for pt in window) / count,
            sum(pt["position"][1] for pt in window) / count,
            sum(pt["position"][2] for pt in window) / count
        ]

        # Use the quaternion from the center of the window (to avoid invalid quat averaging)
        center_pt = raw_points[i]

        smoothed_points.append({
            "timestamp": center_pt["timestamp"],
            "position": avg_pos,
            "quaternion": center_pt["quaternion"]
        })

    return smoothed_points


def save_to_csv(filepath, trajectory_points):
    """
    Saves the computed trajectory waypoints to a CSV file.
    trajectory_points: list of dicts containing:
      - 'cartesian': [X, Y, Z, RX, RY, RZ]
      - 'joints': [J1, J2, J3, J4, J5, J6]
    """
    headers = [
        "Point_Name", "X", "Y", "Z", "RX", "RY", "RZ",
        "J1", "J2", "J3", "J4", "J5", "J6", "Type"
    ]

    with open(filepath, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(headers)

        for idx, pt in enumerate(trajectory_points):
            pt_name = pt.get("name", f"Path_Pt_{idx+1:03d}")
            cart = pt["cartesian"]
            joints = pt["joints"]
            # Type: 0 represents an absolute Cartesian point in base coordinates
            point_type = 0

            row = [
                pt_name,
                f"{cart[0]:.3f}", f"{cart[1]:.3f}", f"{cart[2]:.3f}",
                f"{cart[3]:.3f}", f"{cart[4]:.3f}", f"{cart[5]:.3f}",
                f"{joints[0]:.4f}", f"{joints[1]:.4f}", f"{joints[2]:.4f}",
                f"{joints[3]:.4f}", f"{joints[4]:.4f}", f"{joints[5]:.4f}",
                point_type
            ]
            writer.writerow(row)

    print(f"[INFO] Trajectory successfully saved to: {filepath}")


def load_from_csv(filepath):
    """
    Loads a trajectory from a CSV file.
    Returns: list of dicts with 'name', 'cartesian', 'joints', and 'type'.
    """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Trajectory file not found: {filepath}")

    points = []
    with open(filepath, mode="r") as file:
        reader = csv.DictReader(file)
        for row in reader:
            points.append({
                "name": row["Point_Name"],
                "cartesian": [
                    float(row["X"]), float(row["Y"]), float(row["Z"]),
                    float(row["RX"]), float(row["RY"]), float(row["RZ"])
                ],
                "joints": [
                    float(row["J1"]), float(row["J2"]), float(row["J3"]),
                    float(row["J4"]), float(row["J5"]), float(row["J6"])
                ],
                "type": int(row["Type"])
            })
    return points


def calculate_velocities_and_scaling(trajectory_points, timestamps):
    """
    Calculates joint velocities between waypoints and computes a velocity
    scaling override parameter to ensure joint speed limits are respected.
    
    trajectory_points: list of dicts with 'joints' [J1..J6]
    timestamps: list of floats (seconds)
    Returns: float (0.01 to 1.0) velocity scaling factor for robot commands
    """
    if len(trajectory_points) < 2:
        return 1.0

    max_exceeded_ratio = 0.0

    for i in range(1, len(trajectory_points)):
        dt = timestamps[i] - timestamps[i-1]
        if dt <= 0:
            continue

        joints_curr = trajectory_points[i]["joints"]
        joints_prev = trajectory_points[i-1]["joints"]

        for j in range(6):
            diff = abs(joints_curr[j] - joints_prev[j])
            speed = diff / dt  # degrees/second
            limit = config.MAX_JOINT_SPEEDS[j]
            
            ratio = speed / limit
            if ratio > max_exceeded_ratio:
                max_exceeded_ratio = ratio

    # Compute required scale. If max speed is within limits, scale is 1.0.
    # If speed is exceeded, we scale down the velocity.
    if max_exceeded_ratio > 1.0:
        scale = 1.0 / max_exceeded_ratio
        # Keep scale in reasonable bounds
        scale = max(0.01, min(scale, 1.0))
        print(f"[WARNING] Trajectory exceeds maximum joint speeds. Required scaling factor: {scale:.2f}")
        return scale * config.SPEED_LIMIT_SCALE
    
    return config.SPEED_LIMIT_SCALE


def update_csv_ik(filepath, kinematics_engine):
    """
    Helper function to recalculate the J1-J6 columns for an existing CSV file
    based on the edited X, Y, Z, RX, RY, RZ columns.
    """
    print(f"[INFO] Recalculating IK for CSV: {filepath}")
    points = load_from_csv(filepath)
    updated_points = []

    for pt in points:
        try:
            joints = kinematics_engine.calculate_ik(pt["cartesian"])
            updated_points.append({
                "cartesian": pt["cartesian"],
                "joints": joints
            })
        except Exception as e:
            print(f"[ERROR] Failed to recalculate IK for point {pt['name']}: {e}")
            # Keep original joints if IK fails
            updated_points.append({
                "cartesian": pt["cartesian"],
                "joints": pt["joints"]
            })

    save_to_csv(filepath, updated_points)
    print("[INFO] CSV update complete.")
