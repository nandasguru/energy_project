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

        print(f"[TEST] Lookup table parameters:  daily window number = {dwin}, day of week = {dow}, power bin index = {pidx}\n")
        
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

# Function to send an alert back to VM once it's been detected
def send_alert_VM(URL):
    ts_ns = time.time_ns()

    vm_line = f'energy_alert{{msg-"hi"}} 1 {ts_ns}'

    try:
        response = requests.post(URL, data=vm_line)
        if response.status_code == 204:
            print("[TEST] Msg send successfully!")
        else:
            print(f"Failed. Status Code: {response.status_code}")
    except:
        pass

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
    metric_VibValue = "dataESP_VibValue"
    metric_VibBool = "dataESP_Vibration"
    
    # Need to specify this is the metric name since it uses parentheses, because usually
    # parentheses denote a function qhile querying
    metric_Temp = "{__name__=\"dataESP_Temp(F)\"}"



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
    #
    #
    
    # For PST time
    pst = pytz.timezone('US/Pacific')
    # Set a custom time. Ignore if using current time (It'll get overwritten no biggie)
    # Set 'end' for when the data starts
    # Just keep 'start' the same as 'end'
    start = user_to_unix_timestamp(8, 13, 2025, 13, 0, 0)
    end = user_to_unix_timestamp(8, 13, 2025, 13, 0, 0)


    # Decides whether to collect real time (True) or historical (False) data
    use_live_data = False
    # When collecting real data, use 60
    step = 60


    # This variable keeps track of the current time window in minutes
    current_time_window = 15
    # Set the index of time window (current minute within window)
    if use_live_data:
        timeNow = datetime.datetime.now()
        time_window_index = timeNow.minute % current_time_window
    # Make sure when doing this, the start/end time is a proper window start (:00, :15, :30, :45)
    else:
        time_window_index = 0


    # Victoria Metrics Variables, for writing alerts to VM
    VM_URL = "http://localhost:8428/write"


    # Experiment 1 Variables
    #
    #

    # Define variables used for comparison
    # Generate alert if power outside of percentile range 5 times in a row or
    # whatever it's set to [percentile_range_threshold], within the 15 minute window
    percentile_range_threshold = 5
    under_05_in_a_row = 0
    over_95_in_a_row = 0
    LOOKUP_PARQUET = pathlib.Path("parquet/727935-24234__8641_8761_8891_9006_9251_power_lookup_table.parquet")
    BIN_EDGES_NPY = pathlib.Path("parquet/727935-24234__8641_8761_8891_9006_9251_power_bin_edges.npy")
    lookup = QuantileLookupPandas(LOOKUP_PARQUET, BIN_EDGES_NPY)

    # Initialize
    value_for_lookup_input = 0
    total_window_sum = 0
    p05 = 0
    p95 = 0





    # Experiment 5 variables
    #
    #
    
    # If the apparent power is over 95th pctl, but still within proper tranformer capacity limit
    # DON'T send alert
    transformer_capacity = None  # Implement later





    # Experiment 6 variables
    #
    #

    # If temperature is above threshold, send alert
    temperature_threshold_F = 90





    # Experiment 7 variables
    #
    #

    # If vibration is above custom theshold, send alert (default threshold is 20)
    use_custom_vibration_threshold = False
    custom_vibration_threshold = 50





    # Start the main loop
    while True:

        print("--------------------------------------------------------------------------------------------------------------")

        # Remove later
        one_or_zero_random = random.choices([1,0], weights=[63,37])[0]
        # print([TEST] one_or_zero_random)


        if time_window_index == 0:
            print("[TEST] ********** Time window start **********\n")

        time_window_index += 1
        print(f"[TEST] Time window index: {time_window_index}\n")


        # Gets current time
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


        # ==================== ALL TEST CASES SHOULD GO HERE ====================

        # TEST CASE 1
        """
        This test case checks if an apparent power is outside the percentile range
        (< 5th percentile or > 95th percentile). If it is outside this range
        [percentile_range_threshold] times (default is 5 I guess), send alert. Otherwise
        don't have to do anything
        """

        # Step 1:
        # Get data for the minute
        # Get timestamp and value of apparent power
        # Use timestamp and lookup table input for the time window (only first minute of each window)
        # Initialize total window sum = 0 (only first minute of each window)
        #

        
        
        # Get data
        dataS1 = query_vm_range(metric_S1, start, end, step)
        dataS2 = query_vm_range(metric_S2, start, end, step)

        # Only continue if the data exists
        if dataS1 and dataS2:
            ts_S1, valS1 = dataS1[-1] # Get latest data point for S1
            ts_S2, valS2 = dataS2[-1] # Get latest data point for S2

            # Get time stamp for App.Power. Use S1 or S2, doesn't matter, same timestamp
            dt_S = datetime.datetime.fromtimestamp(ts_S1, pytz.utc).astimezone(pst)
            dt_S_str = dt_S.strftime("%Y-%m-%d %H:%M:%S %Z")

            # valS1 and valS2 are in W, first convert to kW
            valS1 /= 1000.0
            valS2 /= 1000.0
            
            # Now have the timestamp and data separate, can use timestamp for lookup table
            # to get predicted values for the time window and initialize totaw window sum
            # Only needs to be done once at the beginning of each time window

            # Print current timestamp
            print(f"\n[TEST] Current timestamp: {dt_S_str}\n")
            
            if time_window_index == 1: # Beginning of time window
                
                p05, p95 = lookup.predict(dt_S, value_for_lookup_input)
                # Now have p05 and p95 for the time window

                # Initialize total window sum = 0
                total_window_sum = 0
                print(f"[TEST] Set total window sum to {total_window_sum}")

            # Print p05, p95 for each minute, even though it's only updated each window
            print(f"5th pctl value: {p05:.3f}, 95th pctl value: {p95}")


            # Step 2:
            # Add total window sum by App.power each minute (for later use)
            # Compare each App.Power value to p05 and p95 values for time window
            # If App.Power exceeds any value 5 times in a row within time window, alert
            #


            # Increment total window sum by App.Power value
            # ==========Still need to decide whether to use 1 phase or both combined. Use S1 for now==========
            S = valS1
            #S = valS2
            #S = valS1 + valS2

            total_window_sum += S # Will be used later for next window lookup table
            print(f"[TEST] Current total window sum: {total_window_sum:.5f}\n")

            # Now start the comparisions for raisng alerts

            # First case: Under 5th pctl
            if S < p05:
                under_05_in_a_row += 1 # Increase under 5 pctl count
                over_95_in_a_row = 0 # Reset over 95 pctl count

                # If under 5 pctl [percentile_range_threshold] time in a row, alert
                if under_05_in_a_row == percentile_range_threshold:
                    print("==========THIS IS THE ALERT (Under 05th pctl)==========\n")
                    under_05_in_a_row = 0 # Reset count once alert occurs

            # Second case: Over 95th pctl
            elif S > p95:
                over_95_in_a_row += 1 # Increase over 95 pctl count
                under_05_in_a_row = 0 # Reset under 5 pctl count

                # If over 95 pctl [percentile_range_threshold] times in a row, alert
                if over_95_in_a_row == percentile_range_threshold:
                    print("==========THIS IS THE ALERT (Over 95th pctl)==========\n")
                    over_95_in_a_row = 0 # Reset count once alert occurs

            # Last case: Within range
            else:
                # Just reset the count, since nothing out of range
                under_05_in_a_row = 0
                over_95_in_a_row = 0

            # This concludes the comparision for out of range App.Power, but still need
            # to make sure that all this happens within the same time window
            # If time window is about to change, just reset the counts to 0
            # Also do Step 4 checks in same if statement

            if time_window_index == current_time_window:
                under_05_in_a_row = 0
                over_95_in_a_row = 0

            
            # At this point all cases SHOULD be taken care of, all within the same time window

            # Step 3:
            # This just happens every minute, taken care of by this while loop
            #

            # Step 4:
            # This happens when the end of the time window is reached / last minute of time window
            # Once Step 2 finishes for last minute:
            # Total window sum should be the sum of all App.Power values from time window
            # Value for the next window lookup table input is calculated
            # Reset total window sum to 0 to keep it within the time window
            #

            
            # All of this happens after Step 2 finishes for final minute, which it should have by this point
            # Make sure it's the final mintue
                
                # Check total window sum
                print(f"[TEST] Total window sum: {total_window_sum:.3f}")

                # Set value for the next window lookup table
                value_for_lookup_input = total_window_sum * 0.25 # Total window sum x factor
                print(f"[TEST] Value for the next window lookup table: {value_for_lookup_input}")

                # Reset total window sum to 0 so it only sums values for one window
                total_window_sum = 0

            # At this point, data should be coming in every minute, being compared to the correct values
            # Lookup table inputs and total window sums should be calculated correctly
            # p05 and p95 are calculated once at the beginning of each time window
            
            # Print current counts and checks for debugging
            print(f"[TEST] Under 5 count:   {under_05_in_a_row}")
            print(f"[TEST] Over 95 count:   {over_95_in_a_row}\n")

        # Skip everything and print warning if no data is found
        else:
            print("[WARNING] No data received")
            break    


        # TEST CASE 6
        """
        This test case just monitors temperature read by the temperature sensor, and sends
        and alert if the temperature (in F) reaches a value higher than the specified
        threshold [temperature_threshold_F]
        """
        dataTemp = query_vm_range(metric_Temp, start, end, step)

        if dataTemp:
            ts_Temp, valTemp = dataTemp[-1]

            # Temp timestamp, should be same as the app.power timestamp, but just in case
            dt_Temp_str = datetime.datetime.fromtimestamp(ts_Temp, pytz.utc).astimezone(pst).strftime("%Y-%m-%d %H:%M:%S %Z")

            # Print the values
            print(f"[TEST] Timestamp: {dt_Temp_str}, Temperature: {valTemp}")

            # Check for alerts
            if valTemp > temperature_threshold_F:
                print("==========THIS IS THE ALERT (Excess Temperature)========== ")

        else:
            print("[WARNING] No data received")
            break



        # TEST CASE 7: High vibration
        """
        This test case monitors the vibration read by the accelerometer, and sends an alert
        if the vibration is above 20 (default setting) or a custom variable
        [custom_vibration_threshold]
        """
        dataVibrationBoolean = query_vm_range(metric_VibBool, start, end, step)
        dataVibrationMagnitude = query_vm_range(metric_VibValue, start, end, step)

        if dataVibrationBoolean and dataVibrationMagnitude:
            
            ts_VibB, valVibB = dataVibrationBoolean[-1]
            ts_VibM, valVibM = dataVibrationMagnitude[-1]

            # Vibration timestamp, should be the same as app.power timestamp, but just in case
            dt_Vib_str = datetime.datetime.fromtimestamp(ts_VibB, pytz.utc).astimezone(pst).strftime("%Y-%m-%d %H:%M:%S %Z")

            # Print the values
            print(f"[TEST] Timestamp: {dt_Vib_str}, VibrationMag: {valVibM}, VibrationBool: {valVibB}")

            # Check for alert
            if use_custom_vibration_threshold:
                if valVibM > custom_vibration_threshold:
                    print("==========THIS IS AN ALERT (Excess Vibration)==========")
            else:
                if valVibB:
                    print("==========THIS IS AN ALERT (Excess Vibration)==========")
                    
        else:
            print("[WARNING] No data received")
            break

        # ==================== ALL TEST CASES SHOULD END HERE ====================

        # When using historical data, self increment start and end
        if not use_live_data: 
            start += step
            end += step

        if time_window_index == current_time_window:
            print("[TEST] ********** Time window end **********\n")
            time_window_index = 0
        
        time.sleep(step if use_live_data else 10)
        
