# capture.py
"""
Data capture module interfacing with libsurvive (pysurvive) to record tracker poses.
"""

import sys
import time
import math
import msvcrt
import config

print("capture.py loaded")

class TrackerDataCapture:
    def __init__(self):
        self.context = None
        self.recorded_points = []
        self.tracker_name = None

    def initialize_tracker(self):
        """Initializes the libsurvive context and finds the active tracker."""
        print("[INFO] Initializing libsurvive context...")
        # Initialize the survive context with system arguments
        self.context = pysurvive.SimpleContext(sys.argv)
        
        # Search for available tracked objects (HMD, Controllers, Trackers)
        devices = list(self.context.Objects())
        if not devices:
            raise RuntimeError("No tracked devices detected. Ensure your HTC Vive tracker/controller is powered on and connected.")
        
        # Prefer controllers or trackers over the HMD for teaching
        for device in devices:
            name = device.Name().lower()
            if "tracker" in name or "controller" in name or "wm" in name:
                self.tracker_name = device.Name()
                print(f"[INFO] Selected teaching device: {self.tracker_name}")
                break
        
        if not self.tracker_name:
            self.tracker_name = devices[0].Name()
            print(f"[WARNING] No controller/tracker identified. Defaulting to first device: {self.tracker_name}")

    def capture_loop(self):
        """
        Runs the capture loop. Recording starts immediately.
        Stops when the user presses any key in the terminal.
        """
        if not self.context:
            self.initialize_tracker()

        self.recorded_points = []
        start_time = time.time()
        
        print("\n=======================================================")
        print(">>> RECORDING STARTED <<<")
        print("Move the tracker to demonstrate the path.")
        print("Press any key in the terminal (e.g. Enter) to STOP recording.")
        print("=======================================================\n")

        # Clear any existing key strokes in the stdin buffer
        while msvcrt.kbhit():
            msvcrt.getch()

        sampling_interval = 1.0 / config.SAMPLING_RATE_HZ
        next_sample_time = time.time()
        last_pos = None
        MAX_JUMP_M = 0.05  # 50mm maximum displacement per sample at 50Hz

        while self.context.Running():
            # Check if user pressed a key to stop recording
            if msvcrt.kbhit():
                msvcrt.getch() # Consume the key press
                print("\n[INFO] Stop key detected. Stopping recording...")
                break

            current_time = time.time()
            # Maintain the target sampling rate
            if current_time >= next_sample_time:
                # Update survive context to retrieve new packets
                latest_pose = None
                while True:
                    updated = self.context.NextUpdated()
                    if not updated:
                        break
                    if updated.Name() == self.tracker_name:
                        latest_pose = updated.Pose()
                        
                if latest_pose:
                    # pose.Pos is [x, y, z] in meters
                    # pose.Rot is [qw, qx, qy, qz] (w, x, y, z format in libsurvive)
                    pos = [latest_pose.Pos[0], latest_pose.Pos[1], latest_pose.Pos[2]]
                    rot = [latest_pose.Rot[0], latest_pose.Rot[1], latest_pose.Rot[2], latest_pose.Rot[3]]
                    
                    is_valid = True
                    if last_pos is not None:
                        dist = math.sqrt(sum((a - b)**2 for a, b in zip(pos, last_pos)))
                        if dist > MAX_JUMP_M:
                            is_valid = False
                            print(f"[WARNING] Tracker jump detected ({dist:.3f}m). Ignoring frame.")
                    
                    if is_valid:
                        self.recorded_points.append({
                            "timestamp": current_time - start_time,
                            "position": pos,
                            "quaternion": rot
                        })
                        last_pos = pos
                
                next_sample_time = current_time + sampling_interval
            
            # Yield CPU slice
            time.sleep(0.001)

        print(f"[INFO] Recording stopped. Captured {len(self.recorded_points)} raw tracking points.")
        return self.recorded_points
