# config.py
"""
Configuration settings for the Programming-Free Teaching System.
"""

import os

# --- Fairino Simulator Settings ---
# Default IP address for the FAIRINO SimMachine virtual machine / container
ROBOT_IP = "192.168.58.2"

# --- Tracking Settings ---
# Sampling frequency for querying tracker poses (in Hertz)
SAMPLING_RATE_HZ = 50.0

# --- Trajectory Storage Settings ---
# Directory to store generated trajectory CSV files
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trajectories")
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Default filename for the output path
DEFAULT_CSV_FILENAME = "spray_path.csv"

# --- Motion Filtering & Smoothing ---
# Moving average window size for smoothing raw Cartesian coords (1 = no filtering)
FILTER_WINDOW_SIZE = 5

# --- Robot Limits & Playback ---
# Maximum joint speeds for J1 to J6 in degrees per second (FR5 cobot specs)
MAX_JOINT_SPEEDS = [180.0, 180.0, 180.0, 180.0, 180.0, 180.0]

# Neutral joint configuration (home/safe pose) for start and end transitions
NEUTRAL_JOINT_POSE = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]

# Safety margin scaling factor for playback speed (0.1 to 1.0)
SPEED_LIMIT_SCALE = 0.8
