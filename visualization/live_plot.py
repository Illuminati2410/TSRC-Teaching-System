import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ==========================================
# SETTINGS (must match make_trajectory.py)
# ==========================================

JUMP_THRESHOLD_MM = 100
SMOOTH_WINDOW = 10

TARGET_X_SPAN = 150
TARGET_Y_SPAN = 150
TARGET_Z_SPAN = 100

ROBOT_CENTER_X = 0
ROBOT_CENTER_Y = -250
ROBOT_CENTER_Z = 350

# ==========================================
# FIGURE
# ==========================================

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# ==========================================
# UPDATE
# ==========================================

def update(frame):

    ax.clear()

    try:

        df = pd.read_csv("tracker_data.csv")

        if len(df) < 5:
            return

        df = df.tail(500)

        # =====================================
        # RAW TRACKER DATA (meters -> mm)
        # =====================================

        x_raw = df["x"] * 1000
        y_raw = df["y"] * 1000
        z_raw = df["z"] * 1000

        tracker = pd.DataFrame({
            "x": x_raw,
            "y": y_raw,
            "z": z_raw
        })

        # =====================================
        # REMOVE TELEPORTS
        # =====================================

        jump = np.sqrt(
            tracker["x"].diff()**2 +
            tracker["y"].diff()**2 +
            tracker["z"].diff()**2
        )

        tracker.loc[
            jump > JUMP_THRESHOLD_MM,
            ["x", "y", "z"]
        ] = np.nan

        # =====================================
        # SMOOTH
        # =====================================

        tracker["x"] = tracker["x"].rolling(
            SMOOTH_WINDOW,
            min_periods=1
        ).mean()

        tracker["y"] = tracker["y"].rolling(
            SMOOTH_WINDOW,
            min_periods=1
        ).mean()

        tracker["z"] = tracker["z"].rolling(
            SMOOTH_WINDOW,
            min_periods=1
        ).mean()

        valid = tracker.dropna()

        if len(valid) < 5:
            return

        # =====================================
        # SAME TRANSFORM AS make_trajectory.py
        # =====================================

        xmin = valid["x"].min()
        xmax = valid["x"].max()

        ymin = valid["y"].min()
        ymax = valid["y"].max()

        zmin = valid["z"].min()
        zmax = valid["z"].max()

        xspan = xmax - xmin
        yspan = ymax - ymin
        zspan = zmax - zmin

        sx = TARGET_X_SPAN / xspan if xspan > 0 else 1
        sy = TARGET_Y_SPAN / yspan if yspan > 0 else 1
        sz = TARGET_Z_SPAN / zspan if zspan > 0 else 1

        cx = (xmin + xmax) / 2
        cy = (ymin + ymax) / 2
        cz = (zmin + zmax) / 2

        robot_x = (
            (valid["x"] - cx) * sx
            + ROBOT_CENTER_X
        )

        robot_y = (
            (valid["y"] - cy) * sy
            + ROBOT_CENTER_Y
        )

        robot_z = (
            (valid["z"] - cz) * sz
            + ROBOT_CENTER_Z
        )

        # =====================================
        # TRACKER PATH
        # =====================================

        ax.plot(
            valid["x"],
            valid["y"],
            valid["z"],
            color="blue",
            linewidth=2,
            label="Tracker (raw mm)"
        )

        # =====================================
        # ROBOT PATH
        # =====================================

        ax.plot(
            robot_x,
            robot_y,
            robot_z,
            color="red",
            linewidth=3,
            label="Robot path"
        )

        # =====================================
        # CURRENT TRACKER POINT
        # =====================================

        ax.scatter(
            valid["x"].iloc[-1],
            valid["y"].iloc[-1],
            valid["z"].iloc[-1],
            color="blue",
            s=50
        )

        # =====================================
        # CURRENT ROBOT POINT
        # =====================================

        ax.scatter(
            robot_x.iloc[-1],
            robot_y.iloc[-1],
            robot_z.iloc[-1],
            color="red",
            s=80
        )

        # =====================================
        # LABELS
        # =====================================

        ax.set_title(
            "Tracker (Blue) vs Robot Path (Red)"
        )

        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")

        ax.legend()
        ax.grid(True)

        print(
            f"Tracker span: "
            f"X={xspan:.1f} "
            f"Y={yspan:.1f} "
            f"Z={zspan:.1f}",
            end="\r"
        )

    except Exception as e:
        print("Plot Error:", e)

# ==========================================
# ANIMATION
# ==========================================

ani = FuncAnimation(
    fig,
    update,
    interval=100,
    cache_frame_data=False
)

plt.show()