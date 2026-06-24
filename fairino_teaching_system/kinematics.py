# kinematics.py
"""
Kinematics module handling coordinate frame transformations, quaternion math,
and interfacing with Fairino's GetInverseKin RPC API.
"""

import math
import config

# --- Quaternion Utilities ---

def conjugate_quat(q):
    """Returns the conjugate of quaternion q (w, x, y, z)."""
    return [q[0], -q[1], -q[2], -q[3]]

def multiply_quat(q1, q2):
    """Multiplies two quaternions q1 and q2 (w, x, y, z format)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return [w, x, y, z]

def rotate_vector_by_quat(v, q):
    """Rotates a 3D vector v by quaternion q (w, x, y, z)."""
    # qv = [0, vx, vy, vz]
    qv = [0.0, v[0], v[1], v[2]]
    # q_rotated = q * qv * q_conjugate
    q_conj = conjugate_quat(q)
    q_temp = multiply_quat(q, qv)
    q_res = multiply_quat(q_temp, q_conj)
    return [q_res[1], q_res[2], q_res[3]]

def euler_to_quat(roll, pitch, yaw):
    """
    Converts Euler angles in degrees (roll, pitch, yaw) to quaternion [w, x, y, z].
    Uses Z-Y-X rotation order convention.
    """
    r = math.radians(roll)
    p = math.radians(pitch)
    yaw_rad = math.radians(yaw)

    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [qw, qx, qy, qz]

def quat_to_euler(qw, qx, qy, qz):
    """
    Converts a quaternion [w, x, y, z] to Euler angles in degrees (roll, pitch, yaw).
    Uses Z-Y-X rotation order convention.
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


# --- Relative Transformation & IK Solver ---

class KinematicsEngine:
    def __init__(self, robot_client):
        self.robot = robot_client

    def compute_relative_trajectory(self, raw_points, robot_start_pose, scale=1.0):
        """
        Converts raw tracker points to robot Cartesian waypoints relative to the
        operator's initial pose, mapped starting from the robot's initial TCP pose.
        
        raw_points: list of dicts with keys 'position' [x, y, z] (m) and 'quaternion' [w, x, y, z]
        robot_start_pose: list [X0, Y0, Z0, RX0, RY0, RZ0] (mm, degrees)
        scale: workspace scaling factor
        """
        if not raw_points:
            return []

        # Get initial pose of tracker
        start_pos = raw_points[0]["position"]
        start_quat = raw_points[0]["quaternion"]
        start_quat_inv = conjugate_quat(start_quat)

        # Get initial orientation of robot as quaternion
        robot_start_quat = euler_to_quat(robot_start_pose[3], robot_start_pose[4], robot_start_pose[5])

        transformed_waypoints = []

        for pt in raw_points:
            # 1. Translate point so it starts at (0, 0, 0)
            diff_pos = [
                pt["position"][0] - start_pos[0],
                pt["position"][1] - start_pos[1],
                pt["position"][2] - start_pos[2]
            ]
            
            # 2. Rotate relative translation vector into starting tracker's local frame
            local_rel_pos = rotate_vector_by_quat(diff_pos, start_quat_inv)

            # 3. Scale and map relative position (meters to mm) to robot coordinate frame
            robot_x = robot_start_pose[0] + local_rel_pos[0] * 1000.0 * scale
            robot_y = robot_start_pose[1] + local_rel_pos[1] * 1000.0 * scale
            robot_z = robot_start_pose[2] + local_rel_pos[2] * 1000.0 * scale

            # 4. Compute relative orientation: q_rel = q_start^-1 * q_i
            q_rel = multiply_quat(start_quat_inv, pt["quaternion"])

            # 5. Map relative orientation onto robot's start orientation: q_robot = q_robot_start * q_rel
            q_robot = multiply_quat(robot_start_quat, q_rel)
            
            # Convert robot orientation back to Euler angles
            robot_rx, robot_ry, robot_rz = quat_to_euler(q_robot[0], q_robot[1], q_robot[2], q_robot[3])

            transformed_waypoints.append({
                "timestamp": pt["timestamp"],
                "cartesian": [robot_x, robot_y, robot_z, robot_rx, robot_ry, robot_rz]
            })

        return transformed_waypoints

    def calculate_ik(self, cartesian_pose):
        """
        Calls Fairino robot RPC API to calculate Inverse Kinematics for a Cartesian pose.
        cartesian_pose: list [X, Y, Z, RX, RY, RZ]
        Returns: list [J1, J2, J3, J4, J5, J6] (degrees)
        """
        # type=0: Absolute pose (base coordinate system)
        # config=-1: Automatic configuration resolution
        res = self.robot.GetInverseKin(0, cartesian_pose, -1)
        
        # Check if call was successful (error code = 0)
        if isinstance(res, list) and len(res) >= 2 and res[0] == 0:
            return res[1]
        
        raise RuntimeError(f"IK failed for pose {cartesian_pose}. SDK returned: {res}")
