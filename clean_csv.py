import pandas as pd
import numpy as np

df = pd.read_csv("tracker_data.csv")

dx = df["x"].diff()
dy = df["y"].diff()
dz = df["z"].diff()

jump = np.sqrt(dx**2 + dy**2 + dz**2)

df = df[jump < 0.05]     # 50 mm

df.to_csv(
    "tracker_clean.csv",
    index=False
)

print("Saved tracker_clean.csv")