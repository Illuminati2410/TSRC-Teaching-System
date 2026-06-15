import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

def update(frame):

    ax.clear()

    try:
        df = pd.read_csv("steamvr_data.csv")

        if len(df) < 2:
            return

        x = df["x"]
        y = df["y"]
        z = df["z"]

        # trajectory
        ax.plot(x, y, z)

        # current position
        ax.scatter(
            x.iloc[-1],
            y.iloc[-1],
            z.iloc[-1]
        )

        ax.set_title("SteamVR Live Tracker Path")

        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")

    except Exception as e:
        print(e)

ani = FuncAnimation(
    fig,
    update,
    interval=100
)

plt.show()