# robot_control.py
"""
Robot control module to interface with the Fairino simulator RPC server.
"""

import time
import config

try:
    from fairino import Robot
except ImportError:
    # If fairino SDK is not installed in the environment, we raise an ImportError.
    # The user specifies that the script should raise errors if the live setup is not running.
    raise ImportError(
        "fairino library is not installed. Please install the Fairino Python SDK "
        "and add it to your python environment path."
    )


class RobotController:
    def __init__(self, ip=config.ROBOT_IP):
        self.ip = ip
        self.robot = None

    def connect(self):
        """Establishes an RPC connection to the Fairino simulator."""
        print(f"[INFO] Connecting to Fairino robot simulator at {self.ip}...")
        try:
            self.robot = Robot.RPC(self.ip)
            # Perform a simple check to verify connection
            err, pose = self.robot.GetActualTCPPose()
            if err != 0:
                raise ConnectionError(f"Fairino simulator returned error code {err}")
            print("[INFO] Successfully connected to Fairino simulator.")
        except Exception as e:
            raise ConnectionError(f"Could not connect to Fairino simulator at {self.ip}: {e}")

    def get_current_pose(self):
        """
        Queries the current TCP Pose [X, Y, Z, RX, RY, RZ] from the simulator.
        Returns: list of 6 floats (mm and degrees)
        """
        if not self.robot:
            self.connect()
        
        err, pose = self.robot.GetActualTCPPose()
        if err == 0:
            return pose
        else:
            raise RuntimeError(f"Failed to get current TCP pose. Error code: {err}")

    def get_current_joints(self):
        """
        Queries the current joint angles [J1, J2, J3, J4, J5, J6] in degrees.
        """
        if not self.robot:
            self.connect()

        err, joints = self.robot.GetActualJointPosDegree()
        if err == 0:
            return joints
        else:
            raise RuntimeError(f"Failed to get current joint positions. Error code: {err}")

    def execute_trajectory(self, points, playback_mode="joint", speed_scale=1.0):
        """
        Plays back the trajectory points on the robot simulator.
        
        points: list of dicts with 'cartesian' and 'joints'
        playback_mode: 'joint' (uses MoveJ) or 'cartesian' (uses MoveL)
        speed_scale: velocity scale factor (0.01 to 1.0)
        """
        if not self.robot:
            self.connect()

        n_points = len(points)
        print(f"\n[INFO] Starting playback of {n_points} waypoints...")
        print(f"[INFO] Mode: {playback_mode.upper()} | Speed Scale: {speed_scale:.2f}")

        # Set tool and user coordinate system index to 0 (default)
        tool = 0
        user = 0

        # Run trajectory
        for idx, pt in enumerate(points):
            cart = pt["cartesian"]
            joints = pt["joints"]

            # Calculate movement velocity (standard velocity range is 0 to 100%)
            # We scale the default velocity (e.g. 20%) by the computed speed scale
            base_vel = 20.0
            vel = max(1.0, min(100.0, base_vel * speed_scale))
            acc = vel  # Keep acceleration matching velocity

            print(f"[{idx+1}/{n_points}] Executing {pt['name']}...")

            if playback_mode == "joint":
                # MoveJ uses joint angles to move the robot
                # robot.MoveJ(joints, tool, user, vel, acc)
                err = self.robot.MoveJ(joints, tool, user, vel, acc)
            else:
                # MoveL uses Cartesian coordinates to move the robot linearly
                # robot.MoveL(cart, tool, user, vel, acc)
                err = self.robot.MoveL(cart, tool, user, vel, acc)

            if err != 0:
                print(f"[ERROR] Motion command failed at {pt['name']} with error code {err}")
                break

        print("[INFO] Trajectory playback completed.")
