import pandas as pd

df = pd.read_csv("tracker_data.csv")

print("FIRST 10 ROWS")
print(df.head(10))

print("\nLAST 10 ROWS")
print(df.tail(10))

df = df[df["time"] > 5]

print("\nAFTER FILTER (time > 5 sec)")
print("Samples:", len(df))

print("\nX")
print("min =", df["x"].min())
print("max =", df["x"].max())
print("span =", df["x"].max() - df["x"].min())

print("\nY")
print("min =", df["y"].min())
print("max =", df["y"].max())
print("span =", df["y"].max() - df["y"].min())

print("\nZ")
print("min =", df["z"].min())
print("max =", df["z"].max())
print("span =", df["z"].max() - df["z"].min())