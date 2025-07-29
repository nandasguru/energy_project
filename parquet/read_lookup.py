#!/usr/bin/env python3
"""
RPi inference helper for LightGBM quantile lookup tables.

The Parquet file contains:
    daily_window_number (int 1‥96)
    day_of_week         (int 1‥7, ISO: 1=Mon)
    key_feature_value   (float, midpoint of bin – *not* used at inference)
    power_bin_idx       (int 0‥N‑1)
    predicted_05th_pctl (float)
    predicted_95th_pctl (float)

The companion .npy file stores the exact bin edges that were built during
training.  We need them to find the correct `power_bin_idx` for a live sample.
"""
from __future__ import annotations

import pathlib
import time
from datetime import datetime
from typing import Tuple

import numpy as np
import pandas as pd                       #  <‑‑ APPROACH A
# import duckdb                           #  <‑‑ uncomment for APPROACH B

# -----------------------------------------------------------------------------
# CONFIGURATION
# -----------------------------------------------------------------------------
lookup_table_1 = "parquet/720388-469__732_10481_power_lookup_table.parquet"
power_bin_npy_1 = "parquet/720388-469__732_10481_power_bin_edges.npy"
lookup_table_2 = "parquet/720388-469__1001_1681_10421_power_lookup_table.parquet"
power_bin_npy_2 = "parquet/720388-469__1001_1681_10421_power_bin_edges.npy"
LOOKUP_PARQUET = pathlib.Path(lookup_table_2)
BIN_EDGES_NPY  = pathlib.Path(power_bin_npy_2)

# -----------------------------------------------------------------------------
# SMALL UTILITIES
# -----------------------------------------------------------------------------
def daily_window_number(ts: datetime) -> int:
    """Return 1‑based 15‑min slot index within the day (1 .. 96)."""
    return ts.hour * 4 + ts.minute // 15 + 1


def power_bin_index(power: float, edges: np.ndarray) -> int:
    """
    Map *power* to the integer bin index used in the lookup table.
    The table uses `np.digitize() - 1`, so we replicate that logic
    and clip to the valid range.
    """
    idx = np.digitize(power, edges) - 1
    # keep inside [0, N‑1]
    idx = min(max(idx, 0), len(edges) - 2)
    return int(idx)


# -----------------------------------------------------------------------------
# APPROACH A – Pandas in‑RAM DataFrame
# -----------------------------------------------------------------------------
class QuantileLookupPandas:
    """Load the Parquet once, keep it in RAM, answer look‑ups in O(1)."""

    def __init__(self, parquet_path: pathlib.Path, bin_edges_path: pathlib.Path):
        print("[INFO] Loading lookup table …")
        t0 = time.perf_counter()

        self.table: pd.DataFrame = pd.read_parquet(parquet_path)
        self.bin_edges: np.ndarray = np.load(bin_edges_path)

        # (optional) multi‑index → faster filtering
        self.table.set_index(
            ["daily_window_number", "day_of_week", "power_bin_idx"],
            inplace=True,
        )

        elapsed = time.perf_counter() - t0
        print(f"[INFO]   done in {elapsed:.2f}s "
              f"({self.table.memory_usage(deep=True).sum()/1_048_576:.1f} MB)")

    # -------------------------------------------------------------------------
    def predict(
        self,
        ts: datetime,
        total_apparent_power_lag_1: float,
    ) -> Tuple[float, float]:
        """
        Parameters
        ----------
        ts : datetime
            The current timestamp (timezone‑aware is fine).
        total_apparent_power_lag_1 : float
            The most recent 15‑min apparent power average
            (= the *key feature* used to build the table).

        Returns
        -------
        (p05, p95) : tuple of floats
            The 5 th and 95 th percentile predictions in kW.
        """

        dwin = daily_window_number(ts)
        dow  = ts.isoweekday()           # 1=Mon … 7=Sun
        pidx = power_bin_index(total_apparent_power_lag_1, self.bin_edges)

        try:
            row = self.table.loc[(dwin, dow, pidx)]
        except KeyError:
            raise ValueError(
                f"No lookup row for window={dwin}, dow={dow}, bin={pidx}"
            ) from None

        return float(row["predicted_05th_pctl"]), float(row["predicted_95th_pctl"])


# -----------------------------------------------------------------------------
# APPROACH B – DuckDB zero‑copy SQL (OPTIONAL)
# -----------------------------------------------------------------------------
class QuantileLookupDuckDB:
    """
    Same API as the Pandas version but uses DuckDB 0‑copy mapping.  Handy if
    you already rely on SQL elsewhere and want to avoid loading the full
    table into Python memory.
    """

    def __init__(self, parquet_path: pathlib.Path, bin_edges_path: pathlib.Path):
        import duckdb  # local import so the module remains optional

        self.con = duckdb.connect(database=":memory:")
        self.con.execute(
            "CREATE TEMPORARY VIEW lookup AS "
            f"SELECT * FROM '{parquet_path.as_posix()}'"
        )
        self.bin_edges = np.load(bin_edges_path)

    # -------------------------------------------------------------------------
    def predict(
        self,
        ts: datetime,
        total_apparent_power_lag_1: float,
    ) -> Tuple[float, float]:

        dwin = daily_window_number(ts)
        dow  = ts.isoweekday()
        pidx = power_bin_index(total_apparent_power_lag_1, self.bin_edges)

        query = (
            "SELECT predicted_05th_pctl, predicted_95th_pctl "
            "FROM lookup "
            "WHERE daily_window_number = ? "
            "  AND day_of_week         = ? "
            "  AND power_bin_idx       = ? "
        )
        result = self.con.execute(query, (dwin, dow, pidx)).fetchone()
        if result is None:
            raise ValueError(
                f"No lookup row for window={dwin}, dow={dow}, bin={pidx}"
            )

        return float(result[0]), float(result[1])


# -----------------------------------------------------------------------------
# DEMONSTRATION
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    # pick one backend
    lookup = QuantileLookupPandas(LOOKUP_PARQUET, BIN_EDGES_NPY)
    # lookup = QuantileLookupDuckDB(LOOKUP_PARQUET, BIN_EDGES_NPY)

    # live sample -------------------------------------------------------------
    now = datetime.now()                 # use aware tz in production
    random_time = datetime(2021, 4, 15, 11, 47)
    recent_apparent_power = 4.7         # kVA (example)

    p05, p95 = lookup.predict(now, recent_apparent_power)
    print(f"[{now:%F %T}] "
          f"recent_apparent_power={recent_apparent_power:.2f} kVA → "
          f"P05={p05:.2f} kW,  P95={p95:.2f} kW")