import pandas as pd

df = pd.read_csv("tracker_trajectory.csv")

print("X min =", df["x"].min())
print("X max =", df["x"].max())

print("Y min =", df["y"].min())
print("Y max =", df["y"].max())

print("Z min =", df["z"].min())
print("Z max =", df["z"].max())