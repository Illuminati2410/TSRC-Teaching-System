import os

import matplotlib.pyplot as plt
import pandas as pd


BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INPUT_FILE = os.path.join(BASE_DIR, "data", "tracker_data.csv")
OUTPUT_FILE = os.path.join(BASE_DIR, "visualization", "orientation_rpy.png")


df = pd.read_csv(INPUT_FILE)

required_columns = [
    "time",
    "raw_roll",
    "raw_pitch",
    "raw_yaw",
    "unwrapped_roll",
    "unwrapped_pitch",
    "unwrapped_yaw",
]
missing_columns = [col for col in required_columns if col not in df.columns]

if missing_columns:
    raise ValueError(f"Missing columns in tracker CSV: {missing_columns}")

if df.empty:
    raise ValueError("tracker_data.csv has no samples")

t = df["time"] - df["time"].iloc[0]

fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8))

plots = [
    ("Roll", "raw_roll", "unwrapped_roll"),
    ("Pitch", "raw_pitch", "unwrapped_pitch"),
    ("Yaw", "raw_yaw", "unwrapped_yaw"),
]

for ax, (label, raw_col, unwrapped_col) in zip(axes, plots):
    ax.plot(t, df[raw_col], label=f"Raw {label}", alpha=0.65)
    ax.plot(t, df[unwrapped_col], label=f"Unwrapped {label}", linewidth=1.8)
    ax.set_ylabel(f"{label} (deg)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

axes[-1].set_xlabel("Time (s)")
fig.suptitle("Raw RPY vs Unwrapped RPY")
fig.tight_layout()

plt.savefig(OUTPUT_FILE, dpi=150)
print(f"Saved: {OUTPUT_FILE}")
