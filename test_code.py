import subprocess

cmd = [
    r"C:\Users\Shaurya\libsurvive\build-win\Release\survive-cli.exe",
    "--record-stdout"
]

proc = subprocess.Popen(
    cmd,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True
)

import time

def get_latest_pose():

    latest = None
    end_time = time.time() + 1.0

    while time.time() < end_time:

        line = proc.stdout.readline()

        if "LH_POSE" in line:
            latest = line.strip()

    return latest

print("Keep tracker flat.")
input("Press Enter for START...")

start_pose = get_latest_pose()

print("\nRotate tracker ~90 deg LEFT YAW.")
input("Press Enter for END...")

end_pose = get_latest_pose()

print("\nSTART:")
print(start_pose)

print("\nEND:")
print(end_pose)

proc.kill()