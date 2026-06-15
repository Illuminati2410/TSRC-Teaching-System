import pandas as pd
import matplotlib.pyplot as plt

# Load CSV
df = pd.read_csv("tracker_data.csv")

# Create 3D figure
fig = plt.figure(figsize=(8,6))
ax = fig.add_subplot(111, projection='3d')

# Plot trajectory
ax.plot(
    df["x"],
    df["y"],
    df["z"],
    linewidth=2
)

# Mark start point
ax.scatter(
    df["x"].iloc[0],
    df["y"].iloc[0],
    df["z"].iloc[0],
    s=50,
    label="Start"
)

# Mark end point
ax.scatter(
    df["x"].iloc[-1],
    df["y"].iloc[-1],
    df["z"].iloc[-1],
    s=50,
    label="End"
)

ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")

ax.set_title("Tracker Trajectory")
ax.legend()

plt.show()