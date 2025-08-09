import pandas as pd
import numpy as np
import requests
import datetime
import pathlib
import time
import pytz


# Class used to read lookup table
class QuantileLookupPandas:
    def __init__(self, parquet_path: pathlib.Path, bin_edges_path: pathlib.Path):
        print("[INFO] Loading lookup table …")
        self.table: pd.DataFrame = pd.read_parquet(parquet_path)
        self.bin_edges: np.ndarray = np.load(bin_edges_path)
        self.table.set_index(
            ["daily_window_number", "day_of_week", "power_bin_idx"],
            inplace=True,
        )
        print("[INFO] Lookup table loaded.")

    def daily_window_number(self, ts: datetime.datetime) -> int:
        return ts.hour * 4 + ts.minute // 15 + 1

    def power_bin_index(self, power: float) -> int:
        idx = np.digitize(power, self.bin_edges) - 1
        idx = min(max(idx, 0), len(self.bin_edges) - 2)
        return int(idx)

    def predict(self, ts: datetime.datetime, power: float):
        dwin = self.daily_window_number(ts)
        dow = ts.isoweekday()
        pidx = self.power_bin_index(power)
        try:
            row = self.table.loc[(dwin, dow, pidx)]
        except KeyError:
            raise ValueError(f"No lookup row for window={dwin}, dow={dow}, bin={pidx}")
        return float(row["predicted_05th_pctl"]), float(row["predicted_95th_pctl"])

# Function to to create a unix timestamp from human readable format
# Time is ordered in the format: MONTH, DAY, YEAR, HOUR, MINUTE, SECOND
# in PST

def user_to_unix_timestamp(month, day, year, hour, minute, second):
    pst = pytz.timezone('US/Pacific')
    dt_machine_local = datetime.datetime(year, month, day, hour, minute, second)
    dt_pst = pst.localize(dt_machine_local)
    dt_utc_for_unix_timestamp = dt_pst.astimezone(pytz.utc)
    return int(dt_utc_for_unix_timestamp.timestamp())

print("[TEST] Epoch Timestamp: ", user_to_unix_timestamp(8, 6, 2025, 18, 30, 0))

# Function to query Victoria Metrics to get 1 data point per minute for each 
# 15 minute time window
def query_vm_range(metric_name, start_unix, end_unix, step):
    url = "http://localhost:8428/api/v1/query_range"
    params = {
        "query": metric_name,
        "start": start_unix,
        "end": end_unix,
        "step": step # 60s intervals
    }

    try:
        resp = requests.get(url, params=params)
        resp.raise_for_status()
        data = resp.json()

        if data["status"] != "success":
            print("Query failed:", data.get("error", "unknown error"))
            return None
        
        results = data["data"]["result"]
        if not results:
            print(f"No data found for metric {metric_name}")
            return None
        
        # results is a list; each element corresponds to a timeseries
        # Each timeseries contains "values": list of [timestamp, value]
        series = results[0]  # assuming only one timeseries for now
        values = series["values"]

        # Convert timestamps to readable and return pairs
        output = []
        for ts, val in values:
            output.append((int(float(ts)), float(val)))

        return output

    except Exception as e:
        print("Error querying VM:", e)
        return None

# Function to round time down to the nearest minute
# This is to match the time stamps here and in Grafana
def round_down_to_minute(ts):
    return ts - (ts % 60)

if __name__ == "__main__":

    metric = "dataESP_S1"
    pst = pytz.timezone('US/Pacific')

    LOOKUP_PARQUET = pathlib.Path("parquet/720388-469__1001_1681_10421_power_lookup_table.parquet")
    BIN_EDGES_NPY = pathlib.Path("parquet/720388-469__1001_1681_10421_power_bin_edges.npy")
    lookup = QuantileLookupPandas(LOOKUP_PARQUET, BIN_EDGES_NPY)

    # Use these for custom time
    start = user_to_unix_timestamp(8, 7, 2025, 14, 0, 0)
    end = user_to_unix_timestamp(8, 7, 2025, 16, 0, 0)

    step = 60

    # This variable keeps track of the current time window
    current_time_window = 15
    time_window_index = 0

    while True:


        if time_window_index == 0:
            print("[TEST] --- Time window start ---")

        time_window_index += 1





        now = int(time.time())

        # Change how often data comes in
        
        # Use these for using current time
        # start = round_down_to_minute((now-step))
        # end = round_down_to_minute(now)
        

        data = query_vm_range(metric, start, end, step)

        if data:
            ts, val = data[-1] # Get latest data point
            dt = datetime.datetime.fromtimestamp(ts, pytz.utc).astimezone(pst)
            dt_str = dt.strftime("%Y-%m-%d %H:%M:%S %Z")

            try:
                p05, p95 = lookup.predict(dt, val)
                print(f"5 pctl is: {p05}, 95 pctl is: {p95}")
            except ValueError as e:
                print(f"Lookup error: {e}")


            print(f"{dt_str}: {val}")
        else:
            print("[WARNING] No data received")

        start += step
        end += step

        if time_window_index == current_time_window:
            print("[TEST] --- Time window end ---")
            time_window_index = 0


            

        time.sleep(2) # this should be step, change it back then remove this comment
                      # when finished testing
