import pandas as pd

df = pd.read_csv("tracker_data.csv")

print(df.loc[df["x"].idxmax()])
print()
print(df.loc[df["y"].idxmax()])
print()
print(df.loc[df["z"].idxmax()])