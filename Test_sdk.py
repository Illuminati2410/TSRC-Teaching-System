import sys
sys.path.insert(
    0,
    r"C:\Users\Shaurya\libsurvive\build-win\Release\tracker_logger\fairino-python-sdk-v2.1.4_robot3.8.4\windows"
)
from fairino import Robot

print("--- Initializing Fairino Connection ---")
robot = Robot.RPC("192.168.86.128")
robot.is_connect = True 

print("\n[Test] Querying Inverse Kinematics (IK)...")

# Force explicit float numbers and use a guaranteed safe flat orientation facing downward
pose = [260, -400, 300, 180, 0, -90]

try:
    # 0 = Flange base frame, -1 = Auto search posture
    result = robot.GetInverseKin(0, pose, -1)
    print(f"Raw Return Value: {result}")

except Exception as e:
    print(f"Failed to run IK calculation: {e}")

robot.CloseRPC()

