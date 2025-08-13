import pandas as pd
import numpy as np
import requests
import datetime
import pathlib
import time
import pytz

import random # Only for local testing without load simulator


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
        print("[INFO] Lookup table loaded.\n")

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
        "step": step # interval in seconds
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

    # All metrics collected in Victoria Metrics by Raspberry Pi from ESP32
    metric_V1 = "dataESP_V1"
    metric_V2 = "dataESP_V2"
    metric_I1 = "dataESP_I1"
    metric_I2 = "dataESP_I2"
    metric_P1 = "dataESP_P1"
    metric_P2 = "dataESP_P2"
    metric_S1 = "dataESP_S1"
    metric_S2 = "dataESP_S2"
    metric_F1 = "dataESP_F1"
    metric_F2 = "dataESP_F2"
    metric_PH1 = "dataESP_PH1"
    metric_PH2 = "dataESP_PH2"
    metric_Temp = "dataESP_Temp(F)"
    metric_VibValue = "dataESP_VibValue"
    metric_VibBool = "dataESP_Vibration"

    # Experiments:
    #
    #
    # Experiment 1: 5 in a row < 5 pctl or > 95 pctl App.Power withi time window
    # Experiment 2: Too much/less voltage from grid
    # Experiment 3: Too much change in frequency (freq can only decrease)
    # Experiment 4: Too low power factor (cannot be > 1) [S / P]
    # Experiment 5: Capacity of transformer. Don't send alert if capacity is available at S > 95 pctl
    # Experiment 6: High temperature check
    # Experiment 7: High vibration check
    #
    #
    #

    # General variables
    pst = pytz.timezone('US/Pacific')



    # Set a custom time. Ignore if using current time (It'll get overwritten no biggie)
    start = user_to_unix_timestamp(8, 7, 2025, 14, 0, 0)
    end = user_to_unix_timestamp(8, 7, 2025, 16, 0, 0)

    # Decides whether to collect real time (True) or historical (False) data
    use_live_data = True

    # When collecting real data, use 60
    step = 60

    # This variable keeps track of the current time window in minutes
    current_time_window = 15

    # Set the index of time window (current minute within window)
    if use_live_data:
        timeNow = datetime.datetime.now()
        time_window_index = timeNow.minute % current_time_window
    else:
        time_window_index = 0

    # Define variables used for comparison
    # Generate alert if power outside of percentile range 5 times in a row or
    # whatever it's set to [percentile_range_threshold], within the 15 minute window
    percentile_range_threshold = 5
    under_05_in_a_row = 0
    over_95_in_a_row = 0

    LOOKUP_PARQUET = pathlib.Path("parquet/727935-24234__8641_8761_8891_9006_9251_power_lookup_table.parquet")
    BIN_EDGES_NPY = pathlib.Path("parquet/727935-24234__8641_8761_8891_9006_9251_power_bin_edges.npy")
    lookup = QuantileLookupPandas(LOOKUP_PARQUET, BIN_EDGES_NPY)

    while True:

        one_or_zero_random = random.choices([1,0], weights=[63,37])[0]
        # print([TEST]one_or_zero_random)

        if time_window_index == 0:
            print("[TEST] --- Time window start ---\n")

        time_window_index += 1



        now = int(time.time())

        # Only need to use current time for live data        
        if use_live_data:
            # Use these for using current time with 60 step
            if step == 60:   
                start = round_down_to_minute((now-step))
                end = round_down_to_minute(now)
            
            # Any other step, use this
            else:    
                start = now - step
                end = now



        data = query_vm_range(metric_S1, start, end, step)

        if data:
            ts, val = data[-1] # Get latest data point
            dt = datetime.datetime.fromtimestamp(ts, pytz.utc).astimezone(pst)
            dt_str = dt.strftime("%Y-%m-%d %H:%M:%S %Z")

            # val (the input metric_S1 for lookup table) is in W, first convert to kW
            val = val / 1000.0

            
            print("[TEST] Time window index: ", time_window_index)
            print(f"{dt_str}: {metric_S1} = {val:.3f}")

            

            # ==================== ALL TEST CASES SHOULD GO HERE ====================

            # TEST CASE 1
            """
            This test case checks if an apparent power is outside the percentile range
            (< 5th percentile or > 95th percentile). If it is outside this range
            [percentile_range_threshold] times (default is 5 I guess), send alert. Otherwise
            don't have to do anything
            """

            try:
                p05, p95 = lookup.predict(dt, val)     
                print(f"[TEST]5 pctl is: {p05:.3f}, 95 pctl is: {p95:.3f}")

                # Until fake values aren't needed anymore
                # print(f"[TEST]Real {metric_S1} value: {real_val}\n")

            except ValueError as e:
                print(f"Lookup error: {e}")
                break

            # At this point, have both metric_S1 value and 5th/95th percentile values
            # can do comparison(s) now

        
            # Check under 05th percentile
            if val < p05:
                under_05_in_a_row += 1 # increase under 5 pctl count
                over_95_in_a_row = 0 # reset the over 95 pctl count

                # Happened enough times to generate alert
                if under_05_in_a_row == percentile_range_threshold:
                    print("\n==========THIS IS THE ALERT (under 05 pctl)==========\n")
                    under_05_in_a_row = 0 # once alert, reset back to 0

            # Check over 95th percentile
            elif val > p95:
                over_95_in_a_row += 1 # increase over 95 pctl count
                under_05_in_a_row = 0 # reset the under 5 pctl count

                # Happened enough time to generate alert
                if over_95_in_a_row == percentile_range_threshold:
                    print("\n==========THIS IS THE ALERT (over 95 pctl)==========\n")
                    over_95_in_a_row = 0 # once alert, reset back to 0

            # Reset alert counts if metric_S1 is within percentile range
            else:
                over_95_in_a_row = 0
                under_05_in_a_row = 0

            # Although repetitive, probably better to keep everything related next to each other
            if time_window_index == current_time_window:
                # Since we are only checking within each time window, we don't want to give an
                # alert if there are 5 outliers in a row if it crosses a time window (for now)
                # Check if we are going to cross a time window, if yes, reset the counts

                under_05_in_a_row = 0
                over_95_in_a_row = 0

                    
        # ==================== ALL TEST CASES SHOULD END HERE ====================
            # print counts and checks
            print(f"[TEST] under 5 count:     {under_05_in_a_row}")
            print(f"[TEST] over 95 count:     {over_95_in_a_row}\n")     



            
        else:
            print("[WARNING] No data received")
            break

        # When using historical data, self increment start and end
        if not use_live_data: 
            start += step
            end += step

        if time_window_index == current_time_window:
            print("[TEST] --- Time window end ---\n")
            time_window_index = 0
        
        time.sleep(step if use_live_data else 3)
