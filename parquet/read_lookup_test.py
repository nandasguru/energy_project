import polars as pl

df1 = pl.read_parquet("parquet/720388-469__732_10481_power_lookup_table.parquet")
df2 = pl.read_parquet("parquet/720388-469__1001_1681_10421_power_lookup_table.parquet")
df1 = pl.read_parquet("parquet/720388-469__732_10481.parquet")

print("Head of df1:\n")
print(df1.slice(0, 20))

print("Head of df2:\n")
print(df2.slice(0, 20))