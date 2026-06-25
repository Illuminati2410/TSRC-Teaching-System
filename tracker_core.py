"""
TrackerReader: Unified tracker acquisition and processing pipeline.

This module is the single source of truth for all Vive Tracker data.
It handles:
  - Starting/stopping survive-cli
  - Parsing POSE and BUTTON lines
  - Quaternion normalization
  - Tracker stabilization
  - Jump detection and rejection
  - Resynchronization
  - Orientation reference frame
  - Latest pose storage

All consumer applications should use this module exclusively.
"""

import math
import re
import subprocess
import threading
from dataclasses import dataclass
from typing import Optional


@dataclass
class TrackerPose:
    """A single tracker pose sample."""
    x: float              # Position in meters
    y: float
    z: float
    qx: float             # Quaternion (normalized)
    qy: float
    qz: float
    qw: float
    timestamp: float      # Time from survive-cli
    button_pressed: bool
    seq: int              # Sample sequence number


class TrackerReader:
    """
    Thread-safe unified tracker reader.
    
    Usage:
        reader = TrackerReader(survive_exe_path, target_device)
        reader.start()
        
        while running:
            pose = reader.get_latest_pose()
            if pose:
                # Use pose...
                pass
            
            time.sleep(0.01)
        
        reader.stop()
    """

    def __init__(self, survive_exe: str, target_device: str = "WM0"):
        """
        Initialize the tracker reader.
        
        Args:
            survive_exe: Path to survive-cli executable
            target_device: Device name to track (e.g., "WM0")
        """
        self.survive_exe = survive_exe
        self.target_device = target_device
        
        self._proc = None
        self._running = False
        self._thread = None
        
        self._lock = threading.Lock()
        self._latest_pose = None
        self._seq = 0
        
        # Stabilization state
        self._stabilization_samples = []
        self._stabilizing = True
        self._tracker_x0 = None
        self._tracker_y0 = None
        self._tracker_z0 = None
        
        # Jump rejection state
        self._last_x = None
        self._last_y = None
        self._last_z = None
        self._reject_x = None
        self._reject_y = None
        self._reject_z = None
        self._stable_rejects = 0
        
        # Orientation reference (set on first accepted sample)
        self._q_ref_w = None
        self._q_ref_x = None
        self._q_ref_y = None
        self._q_ref_z = None
        
        # Button state
        self._button_pressed = False
        self._button_available = False
        
        # Configuration
        self.MAX_JUMP_MM = 200
        self.RESYNC_AFTER_STABLE_REJECTS = 5
        self.STABILIZATION_SAMPLES = 100
        self.STABILIZATION_TIME_THRESHOLD = 3.0

    def start(self) -> bool:
        """
        Start the tracker reader in a background thread.
        
        Returns:
            True if successfully started, False otherwise
        """
        if self._running:
            return False
        
        try:
            self._proc = subprocess.Popen(
                [self.survive_exe, "--record-stdout"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except Exception as e:
            print(f"Failed to start survive-cli: {e}")
            return False
        
        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        
        return True

    def stop(self):
        """Stop the tracker reader and clean up resources."""
        self._running = False
        
        if self._thread:
            self._thread.join(timeout=2.0)
        
        self._stop_process()

    def get_latest_pose(self) -> Optional[TrackerPose]:
        """
        Get the latest tracker pose.
        
        Returns:
            Latest TrackerPose if available, None otherwise.
            Thread-safe copy.
        """
        with self._lock:
            if self._latest_pose is None:
                return None
            
            # Return a copy to prevent external modification
            pose = self._latest_pose
            return TrackerPose(
                x=pose.x,
                y=pose.y,
                z=pose.z,
                qx=pose.qx,
                qy=pose.qy,
                qz=pose.qz,
                qw=pose.qw,
                timestamp=pose.timestamp,
                button_pressed=pose.button_pressed,
                seq=pose.seq,
            )

    def get_tracker_neutral_reference(self) -> tuple:
        """
        Get the tracker neutral reference position (in meters).
        
        Returns:
            Tuple (x0, y0, z0) or (None, None, None) if not yet calibrated
        """
        with self._lock:
            return (self._tracker_x0, self._tracker_y0, self._tracker_z0)

    def is_button_available(self) -> bool:
        """Check if tracker button state is available."""
        with self._lock:
            return self._button_available

    def _reader_loop(self):
        """Main reader thread loop."""
        if not self._proc or not self._proc.stdout:
            return
        
        # Phase 1: Stabilization
        print("\nTracker stabilization...")
        print("Please hold tracker still...")
        
        while self._stabilizing and self._running:
            line = self._proc.stdout.readline()
            if not line:
                if self._proc.poll() is not None:
                    return
                continue
            
            if "POSE" not in line:
                continue
            
            sample = self._parse_pose_line(line)
            if sample is None:
                continue
            
            # Only use samples after t=3.0
            if sample["t"] < self.STABILIZATION_TIME_THRESHOLD:
                continue
            
            self._stabilization_samples.append(sample)
            
            if len(self._stabilization_samples) >= self.STABILIZATION_SAMPLES:
                self._finalize_stabilization()
                self._stabilizing = False
                print(f"\nTracker Neutral Reference")
                print(f"X0 = {self._tracker_x0:.3f}")
                print(f"Y0 = {self._tracker_y0:.3f}")
                print(f"Z0 = {self._tracker_z0:.3f}")
                print("\nStarting actual tracking...\n")
        
        # Phase 2: Continuous tracking
        print("Starting actual recording...\n")
        while self._running:
            line = self._proc.stdout.readline()
            if not line:
                if self._proc.poll() is not None:
                    break
                continue
            
            # Parse pose
            sample = self._parse_pose_line(line)
            if sample is not None:
                self._process_pose_sample(sample)
                continue
            
            # Parse button
            button_state = self._parse_button_line(line)
            if button_state is not None:
                with self._lock:
                    self._button_available = True
                    self._button_pressed = button_state

    def _finalize_stabilization(self):
        """Compute neutral tracker position from stabilization samples."""
        samples = self._stabilization_samples
        self._tracker_x0 = sum(s["x"] for s in samples) / len(samples)
        self._tracker_y0 = sum(s["y"] for s in samples) / len(samples)
        self._tracker_z0 = sum(s["z"] for s in samples) / len(samples)

    def _process_pose_sample(self, sample: dict):
        """
        Process a pose sample through the full pipeline.
        
        Args:
            sample: Dict with keys t, x, y, z, qx, qy, qz, qw
        """
        t = sample["t"]
        x = sample["x"]
        y = sample["y"]
        z = sample["z"]
        qx = sample["qx"]
        qy = sample["qy"]
        qz = sample["qz"]
        qw = sample["qw"]
        
        # Reject obviously impossible positions
        if abs(x) > 5 or abs(y) > 5 or abs(z) > 5:
            return
        
        # Normalize quaternion
        q_norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if q_norm < 1e-6:
            return
        
        qw /= q_norm
        qx /= q_norm
        qy /= q_norm
        qz /= q_norm
        
        # Set orientation reference on first accepted sample
        with self._lock:
            if self._q_ref_w is None:
                self._q_ref_w = qw
                self._q_ref_x = qx
                self._q_ref_y = qy
                self._q_ref_z = qz
        
        # Jump rejection
        x_mm = x * 1000
        y_mm = y * 1000
        z_mm = z * 1000
        
        if self._last_x is not None:
            dx = abs(x_mm - self._last_x)
            dy = abs(y_mm - self._last_y)
            dz = abs(z_mm - self._last_z)
            
            if dx > self.MAX_JUMP_MM or dy > self.MAX_JUMP_MM or dz > self.MAX_JUMP_MM:
                # Jump detected
                if self._reject_x is None:
                    self._stable_rejects = 1
                else:
                    reject_dx = abs(x_mm - self._reject_x)
                    reject_dy = abs(y_mm - self._reject_y)
                    reject_dz = abs(z_mm - self._reject_z)
                    
                    if (reject_dx <= self.MAX_JUMP_MM and
                        reject_dy <= self.MAX_JUMP_MM and
                        reject_dz <= self.MAX_JUMP_MM):
                        self._stable_rejects += 1
                    else:
                        self._stable_rejects = 1
                
                self._reject_x = x_mm
                self._reject_y = y_mm
                self._reject_z = z_mm
                
                # If we haven't had enough stable rejects, skip this sample
                if self._stable_rejects < self.RESYNC_AFTER_STABLE_REJECTS:
                    return
        
        # Accept this sample
        self._last_x = x_mm
        self._last_y = y_mm
        self._last_z = z_mm
        self._reject_x = None
        self._reject_y = None
        self._reject_z = None
        self._stable_rejects = 0
        
        # Store latest pose (atomically with lock)
        with self._lock:
            self._seq += 1
            self._latest_pose = TrackerPose(
                x=x,
                y=y,
                z=z,
                qx=qx,
                qy=qy,
                qz=qz,
                qw=qw,
                timestamp=t,
                button_pressed=self._button_pressed,
                seq=self._seq,
            )

    def _parse_pose_line(self, line: str) -> Optional[dict]:
        """
        Parse a POSE line from survive-cli output.
        
        survive-cli --record-stdout output format:
          parts[0]=time  parts[1]="POSE"  parts[2]=tag
          parts[3]=tx    parts[4]=ty      parts[5]=tz
          parts[6]=qx    parts[7]=qy      parts[8]=qz    parts[9]=qw
        
        Returns:
            Dict with keys: t, x, y, z, qx, qy, qz, qw
            Or None if parsing fails
        """
        if self.target_device not in line or "POSE" not in line:
            return None
        
        parts = line.strip().split()
        
        try:
            pose_index = parts.index("POSE")
        except ValueError:
            return None
        
        if len(parts) <= pose_index + 7:
            return None
        
        try:
            t = float(parts[0])
            x = float(parts[pose_index + 1])
            y = float(parts[pose_index + 2])
            z = float(parts[pose_index + 3])
            qx = float(parts[pose_index + 4])
            qy = float(parts[pose_index + 5])
            qz = float(parts[pose_index + 6])
            qw = float(parts[pose_index + 7])
        except (ValueError, IndexError):
            return None
        
        return {
            "t": t,
            "x": x,
            "y": y,
            "z": z,
            "qx": qx,
            "qy": qy,
            "qz": qz,
            "qw": qw,
        }

    def _parse_button_line(self, line: str) -> Optional[bool]:
        """
        Parse button state from survive-cli output.
        
        Returns:
            True if button pressed, False if released, None if not button line
        """
        upper = line.upper()
        
        if self.target_device not in upper:
            return None
        
        if "BUTTON" not in upper and "BTN" not in upper:
            return None
        
        if "PRESSED" in upper:
            return True
        if "RELEASED" in upper:
            return False
        
        # Try to parse binary digit
        digits = re.findall(r"(?<![0-9.-])[01](?![0-9.-])", upper)
        if digits:
            return digits[-1] == "1"
        
        return None

    def _stop_process(self):
        """Stop the subprocess."""
        if self._proc is None:
            return
        
        try:
            if self._proc.poll() is None:
                self._proc.terminate()
                try:
                    self._proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self._proc.kill()
        except Exception:
            pass
        finally:
            self._proc = None
