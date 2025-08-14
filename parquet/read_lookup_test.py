import polars as pl
import numpy as np


df1 = pl.read_parquet("parquet/720388-469__732_10481_power_lookup_table.parquet")
df2 = pl.read_parquet("parquet/720388-469__1001_1681_10421_power_lookup_table.parquet")
df1 = pl.read_parquet("parquet/720388-469__732_10481.parquet")

df3 = pl.read_parquet("parquet/727935-24234__8641_8761_8891_9006_9251_power_lookup_table.parquet")
bins = np.load("parquet/727935-24234__8641_8761_8891_9006_9251_power_bin_edges.npy")


"""
print("Head of df1:\n")
print(df1.slice(0, 20))

print("Head of df2:\n")
print(df2.slice(0, 20))
"""
print("Head of df3:\n")
print(df3.slice(0, 20))

print("Bin edges file:\n")
print(bins)