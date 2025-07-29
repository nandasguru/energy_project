import numpy as np
import pandas as pd
from pathlib import Path
"""
# Load the array from the .npy file
my_array = np.load("parquet/720388-469__1001_1681_10421_power_bin_edges.npy")

# Now you can use the array
print("Array loaded from file:")
print(my_array)

# Check its shape and data type
print(f"Shape: {my_array.shape}")
print(f"Data type: {my_array.dtype}")
"""

parquet_path = Path("parquet/720388-469__732_10481_power_lookup_table.parquet")

df = pd.read_parquet(parquet_path)

print("\n[INFO] Head of table:")
print(df.head())

print("\n[INFO] Column summary")
print(df.describe(include='all'))

print("\n[INFO] Number of unique values per column:")
print(df.nunique())

print("\n[INFO] Unique predicted percentiles:")
print(df[['predicted_05th_pctl', 'predicted_95th_pctl']].drop_duplicates())