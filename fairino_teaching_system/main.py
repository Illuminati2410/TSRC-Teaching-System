# main.py
"""
Programming-Free Teaching System - Main Entry Point.
"""

import os
import sys
import json
import config
from capture import TrackerDataCapture
from kinematics import KinematicsEngine
from robot_control import RobotController
import trajectory

def show_menu():
    print("\n=======================================================")
    print("      PROGRAMMING-FREE TEACHING SYSTEM MENU            ")
    print("=======================================================")
    print("1. Record a New Trajectory")
    print("2. Playback an Existing CSV Trajectory")
    print("3. Recalculate Joint Angles (IK) for a CSV File")
    print("4. Reprocess Raw Data (Debug)")
    print("5. Exit")
    print("=======================================================")
    try:
        choice = input("Enter your choice (1-5): ").strip()
        return choice
    except KeyboardInterrupt:
        return "5"

def process_and_save_trajectory(robot_ctrl, kinematics, raw_points, robot_start_pose):
    # 4. Smooth Cartesian tracking coordinates
    print("[INFO] Smoothing captured path positions...")
    smoothed_raw = trajectory.smooth_trajectory_positions(
        raw_points, 
        window_size=config.FILTER_WINDOW_SIZE
    )

    # 5. Compute relative Cartesian coordinates relative to the start pose
    print("[INFO] Computing robot Cartesian coordinates (relative offset)...")
    scale = 1.0  # Default scale factor
    scale_input = input("Enter workspace scale factor (default 1.0): ").strip()
    if scale_input:
        try:
            scale = float(scale_input)
        except ValueError:
            print("[WARNING] Invalid scale. Defaulting to 1.0")

    robot_cartesian_points = kinematics.compute_relative_trajectory(
        smoothed_raw, 
        robot_start_pose, 
        scale=scale
    )

    # 6. Calculate Inverse Kinematics (IK) for each point
    print("[INFO] Resolving Inverse Kinematics (IK) via Fairino SDK...")
    processed_points = []
    timestamps = []
    
    # Calculate Cartesian coordinates for the neutral home joint configuration
    try:
        err, neutral_cart = robot_ctrl.robot.GetForwardKin(config.NEUTRAL_JOINT_POSE)
        if err != 0:
            raise RuntimeError(f"SDK returned error code {err}")
        print(f"[INFO] Resolved Neutral Home Cartesian Pose: {neutral_cart}")
    except Exception as e:
        print(f"[ERROR] Cannot compute Forward Kinematics for Neutral Pose: {e}")
        print("[ERROR] Aborting recording.")
        return

    # Add the neutral starting point
    processed_points.append({
        "name": "Path_Pt_Start_Home",
        "cartesian": neutral_cart,
        "joints": config.NEUTRAL_JOINT_POSE
    })
    
    for idx, pt in enumerate(robot_cartesian_points):
        cart = pt["cartesian"]
        timestamps.append(pt["timestamp"])
        
        try:
            # Query J1-J6 via RPC
            joints = kinematics.calculate_ik(cart)
            processed_points.append({
                "name": f"Path_Pt_{idx+1:03d}",
                "cartesian": cart,
                "joints": joints
            })
        except Exception as e:
            print(f"[WARNING] Pose [{idx+1}/{len(robot_cartesian_points)}] IK failed: {e}")
            # Skip or handle if IK fails. To maintain path completeness,
            # we halt if a crucial segment is unreachable.
            print("[ERROR] Trajectory generation halted due to unreachable coordinate.")
            return

    # Add the neutral ending point
    processed_points.append({
        "name": "Path_Pt_End_Home",
        "cartesian": neutral_cart,
        "joints": config.NEUTRAL_JOINT_POSE
    })

    # Adjust timestamps to include transitions for velocity calculations
    if timestamps:
        timestamps.insert(0, timestamps[0] - 2.0)  # 2-second transition from home
        timestamps.append(timestamps[-1] + 2.0)      # 2-second transition back to home
    else:
        timestamps = [0.0, 2.0]

    # 7. Save trajectory to CSV
    csv_filename = input(f"Enter filename to save CSV (default: {config.DEFAULT_CSV_FILENAME}): ").strip()
    if not csv_filename:
        csv_filename = config.DEFAULT_CSV_FILENAME
    if not csv_filename.endswith(".csv"):
        csv_filename += ".csv"

    csv_path = os.path.join(config.OUTPUT_DIR, csv_filename)
    trajectory.save_to_csv(csv_path, processed_points)

    # 8. Immediate Playback Option
    playback_choice = input("Do you want to play back this trajectory on the simulator now? (y/n): ").strip().lower()
    if playback_choice == 'y':
        speed_scale = trajectory.calculate_velocities_and_scaling(processed_points, timestamps)
        
        mode = input("Select playback mode - 'joint' (smoother) or 'cartesian' (linear) (default: joint): ").strip().lower()
        if mode not in ["joint", "cartesian"]:
            mode = "joint"
            
        robot_ctrl.execute_trajectory(processed_points, playback_mode=mode, speed_scale=speed_scale)

def record_flow(robot_ctrl, kinematics):
    # 1. Query starting pose from the robot as the reference starting coordinate frame
    try:
        print("[INFO] Querying robot starting pose from simulator...")
        robot_start_pose = robot_ctrl.get_current_pose()
        print(f"[INFO] Reference robot starting pose: {robot_start_pose}")
    except Exception as e:
        print(f"[ERROR] Cannot retrieve robot starting pose: {e}")
        print("[ERROR] Aborting recording. Ensure the simulator is running.")
        return

    # 2. Initialize tracking capture
    try:
        capturer = TrackerDataCapture()
        capturer.initialize_tracker()
    except Exception as e:
        print(f"[ERROR] Failed to initialize tracker: {e}")
        return

    # 3. Perform recording
    raw_points = capturer.capture_loop()
    if not raw_points:
        print("[WARNING] No points captured. Recording cancelled.")
        return

    # Save raw points to JSON
    raw_filename = input("Enter filename to save raw JSON data (default: raw_capture.json): ").strip()
    if not raw_filename:
        raw_filename = "raw_capture.json"
    if not raw_filename.endswith(".json"):
        raw_filename += ".json"
    
    raw_path = os.path.join(config.OUTPUT_DIR, raw_filename)
    try:
        with open(raw_path, 'w') as f:
            json.dump({
                "robot_start_pose": robot_start_pose,
                "raw_points": raw_points
            }, f, indent=4)
        print(f"[INFO] Raw capture data saved to {raw_path}")
    except Exception as e:
        print(f"[ERROR] Failed to save raw data: {e}")

    process_and_save_trajectory(robot_ctrl, kinematics, raw_points, robot_start_pose)

def reprocess_raw_flow(robot_ctrl, kinematics):
    print("\nAvailable raw data files:")
    files = [f for f in os.listdir(config.OUTPUT_DIR) if f.endswith(".json")]
    if not files:
        print("[WARNING] No JSON files found in the trajectories folder.")
        return

    for idx, f in enumerate(files):
        print(f"  {idx+1}. {f}")

    file_choice = input("Select file index to reprocess: ").strip()
    try:
        file_idx = int(file_choice) - 1
        if file_idx < 0 or file_idx >= len(files):
            raise ValueError
        filename = files[file_idx]
    except ValueError:
        print("[ERROR] Invalid selection.")
        return

    json_path = os.path.join(config.OUTPUT_DIR, filename)
    try:
        with open(json_path, 'r') as f:
            data = json.load(f)
            robot_start_pose = data["robot_start_pose"]
            raw_points = data["raw_points"]
        print(f"[INFO] Loaded {len(raw_points)} points and starting pose {robot_start_pose} from {filename}")
        
        process_and_save_trajectory(robot_ctrl, kinematics, raw_points, robot_start_pose)
    except Exception as e:
        print(f"[ERROR] Failed to load or process raw data: {e}")

def playback_flow(robot_ctrl):
    print("\nAvailable trajectories:")
    files = [f for f in os.listdir(config.OUTPUT_DIR) if f.endswith(".csv")]
    if not files:
        print("[WARNING] No CSV files found in the trajectories folder.")
        return

    for idx, f in enumerate(files):
        print(f"  {idx+1}. {f}")

    file_choice = input("Select file index to play: ").strip()
    try:
        file_idx = int(file_choice) - 1
        if file_idx < 0 or file_idx >= len(files):
            raise ValueError
        filename = files[file_idx]
    except ValueError:
        print("[ERROR] Invalid selection.")
        return

    csv_path = os.path.join(config.OUTPUT_DIR, filename)
    try:
        points = trajectory.load_from_csv(csv_path)
        # Dummy timestamps for uniform scaling evaluation (or assume 10Hz sampling if not recorded)
        dummy_timestamps = [i * 0.02 for i in range(len(points))] # 50Hz estimation
        speed_scale = trajectory.calculate_velocities_and_scaling(points, dummy_timestamps)
        
        mode = input("Select playback mode - 'joint' (smoother) or 'cartesian' (linear) (default: joint): ").strip().lower()
        if mode not in ["joint", "cartesian"]:
            mode = "joint"

        robot_ctrl.execute_trajectory(points, playback_mode=mode, speed_scale=speed_scale)
    except Exception as e:
        print(f"[ERROR] Failed to load/execute trajectory: {e}")

def recalculate_ik_flow(robot_ctrl, kinematics):
    print("\nAvailable trajectories:")
    files = [f for f in os.listdir(config.OUTPUT_DIR) if f.endswith(".csv")]
    if not files:
        print("[WARNING] No CSV files found.")
        return

    for idx, f in enumerate(files):
        print(f"  {idx+1}. {f}")

    file_choice = input("Select file index to recalculate: ").strip()
    try:
        file_idx = int(file_choice) - 1
        if file_idx < 0 or file_idx >= len(files):
            raise ValueError
        filename = files[file_idx]
    except ValueError:
        print("[ERROR] Invalid selection.")
        return

    csv_path = os.path.join(config.OUTPUT_DIR, filename)
    try:
        trajectory.update_csv_ik(csv_path, kinematics)
    except Exception as e:
        print(f"[ERROR] Failed to update CSV IK: {e}")

def main():
    print("=======================================================")
    print("Welcome to Programming-Free Teaching System for Fairino")
    print("=======================================================")
    
    # 1. Connect to the robot simulator
    robot_ctrl = RobotController()
    try:
        robot_ctrl.connect()
    except Exception as e:
        print(f"[ERROR] Failed to connect to Fairino simulator: {e}")
        print("[ERROR] Please make sure the simulator is active and reachable.")
        sys.exit(1)

    kinematics = KinematicsEngine(robot_ctrl.robot)

    # 2. Main menu loop
    while True:
        choice = show_menu()
        if choice == "1":
            record_flow(robot_ctrl, kinematics)
        elif choice == "2":
            playback_flow(robot_ctrl)
        elif choice == "3":
            recalculate_ik_flow(robot_ctrl, kinematics)
        elif choice == "4":
            reprocess_raw_flow(robot_ctrl, kinematics)
        elif choice == "5":
            print("\nExiting. Thank you!")
            break
        else:
            print("[WARNING] Invalid option. Try again.")

if __name__ == "__main__":
    main()
