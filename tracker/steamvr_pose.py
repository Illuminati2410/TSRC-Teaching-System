import openvr
import csv
import time

# Initialize SteamVR
openvr.init(openvr.VRApplication_Other)

vr = openvr.VRSystem()

# Create CSV
csv_file = open("steamvr_data.csv", "w", newline="")
writer = csv.writer(csv_file)
writer.writerow(["time", "x", "y", "z"])

print("Looking for tracker...")

try:
    while True:

        poses = vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0,
            openvr.k_unMaxTrackedDeviceCount
        )

        tracker_found = False

        for i in range(openvr.k_unMaxTrackedDeviceCount):

            pose = poses[i]

            if not pose.bPoseIsValid:
                continue

            device_class = vr.getTrackedDeviceClass(i)

            if device_class == openvr.TrackedDeviceClass_GenericTracker:

                tracker_found = True

                m = pose.mDeviceToAbsoluteTracking

                x = m[0][3] * 1000  # mm
                y = m[1][3] * 1000
                z = m[2][3] * 1000

                t = time.time()

                writer.writerow([t, x, y, z])
                csv_file.flush()

                print(
                    f"X={x:.1f} mm "
                    f"Y={y:.1f} mm "
                    f"Z={z:.1f} mm"
                )

                break

        if not tracker_found:
            print("Tracker not found")

        time.sleep(0.02)  # ~50 Hz

except KeyboardInterrupt:
    print("\nStopping logger...")

finally:
    csv_file.close()
    openvr.shutdown()