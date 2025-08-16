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
def send_alert_VM(URL, ts_real, alert_code):

    ts_before = int(ts_real - 1)
    ts_after = int(ts_real + 1)


    vm_line = (
        f'Alert code=0 {ts_before}\n'
        f'Alert code={alert_code} {ts_real}\n'
        f'Alert code=0 {ts_after}'
    )

    try:
        response = requests.post(URL, data=vm_line)
        if response.status_code == 204:
            print("[TEST] Alert sent to VM successfully!")
        else:
            print(f"Failed. Status Code: {response.status_code}")
    except Exception as e:
        print(f"Error sending test alert: {e}")









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


    # Define alert codes for sending alerts to Victoria Metrics
    No_Alert            =  0
    High_Temperature    =  1
    High_Vibration      =  2
    Consistent_Under_5  =  3
    Consistent_Over_95  =  4
    Total_Sum_Over_95   =  5
    Over_Voltage        =  6
    Under_Voltage       =  7
    Low_Frequency       =  8

    # Experiments:
    #
    #
    # Experiment 1: 5 in a row < 5 pctl or > 95 pctl App.Power withi time window
    # Experiment 2: Too much/less voltage from grid
    # Experiment 3: Too much change in frequency
    # Experiment 4: Too low power factor (cannot be > 1) [S / P] ========== Don't know if do it or not
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
    # This is for a SINGLE Query, not the whole range
    # Set 'end' for when the data starts
    # Just keep 'start' the same as 'end'
    # Keep it to the time window

    end = user_to_unix_timestamp(8, 16, 2025, 11, 0, 0)
    start = user_to_unix_timestamp(8, 13, 2025, 11, 0, 0)
    


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

    # Change this to decide if alerts are actually sent to VM or just printed
    alerts_are_active = True


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





    # Experiment 2 variables
    #
    #
    
    # Initialize thresholds for voltage
    # +- 5%
    under_voltage_threshold = 114
    over_voltage_threshold = 126

    v_range_threshold = 5

    under_v1_in_a_row = 0
    over_v1_in_a_row = 0
    under_v2_in_a_row = 0
    over_v2_in_a_row = 0



    # Experiment 3 variables
    #
    #
    
    under_frequency_threshold = 59.95
    over_frequency_threshold = 60.05

    f_range_threshold = 5
    under_f_in_a_row = 0
    over_f_in_a_row = 0



    # Experiment 4 variables
    #
    #
    # To be done later



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
    temperature_threshold_F = 90 # Change to whatever needed





    # Experiment 7 variables
    #
    #

    # If vibration is above custom theshold, send alert (default threshold is 20)
    use_custom_vibration_threshold = False
    custom_vibration_threshold = 50





    # Start the main loop
    while True:

        print("--------------------------------------------------------------------------------------------------------------")

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

        # EXPERIMENT 1
        """
        This test case checks if an apparent power is outside the percentile range
        (< 5th percentile or > 95th percentile). If it is outside this range
        [percentile_range_threshold] times (default is 5 I guess), send alert. Otherwise
        don't have to do anything. Also send an alert if the total window sum goes above
        the 95th percentile, even if individual App.power values didn't. Don't need to
        check the same condition for < 5th percentile since the alert will trigger
        earlier with the individual data points under 5th percentile check
        """
        # Indicate values are for S
        print("##########################")
        print("APPARENT POWER")
        print("##########################")
        print("")
        
        # Step 1:
        # Get data for the first minute of time window
        # Get timestamp and value of apparent power
        # Use timestamp and lookup table input for the time window (only first minute of each window)
        # Initialize total window sum = 0 (only first minute of each window)
        #

        
        
        # Get data
        dataS1 = query_vm_range(metric_S1, start, end, step)
        dataS2 = query_vm_range(metric_S2, start, end, step)

        # Only continue if the data exists
        if dataS1 and dataS2:
            # Data comes as a tuple (timestamp, value)
            ts_S1, valS1 = dataS1[-1] # Get latest data point for S1
            ts_S2, valS2 = dataS2[-1] # Get latest data point for S2

            # Get time stamp for App.Power. Use S1 or S2, doesn't matter, same timestamp
            dt_S = datetime.datetime.fromtimestamp(ts_S1, pytz.utc).astimezone(pst)
            dt_S_str = dt_S.strftime("%Y-%m-%d %H:%M:%S %Z") # Human readable string

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

            


            # Step 2:
            # Add total window sum by App.power each minute (for later use)
            # Compare each App.Power value to p05 and p95 values for time window
            # If App.Power exceeds any value 5 times in a row within time window, alert
            #


            # Increment total window sum by App.Power value
            # ==========Still need to decide whether to use 1 phase or both combined. Comment whichever necessary==========
            #S = valS1
            #S = valS2
            S = valS1 + valS2

            total_window_sum += S # Will be used later for next window lookup table
            print(f"[TEST] Current total window sum: {total_window_sum:.5f}\n")

            # Print data, p05, p95 for debugging
            print(f"\n[TEST] S1 = {valS1}\n[TEST] S2 = {valS2}\n[TEST] S1 + S2 = {S:.5f}\n")
            print(f"[TEST] value for lookup table input: {value_for_lookup_input}")
            print(f"\nMetric (S): {S:.5f}, 5th pctl value: {p05:.3f}, 95th pctl value: {p95}\n")

            # Now start the comparisions for raisng alerts

            # First case: Consistently under 5th pctl
            if S < p05:
                under_05_in_a_row += 1 # Increase under 5 pctl count
                over_95_in_a_row = 0 # Reset over 95 pctl count

                # If under 5 pctl [percentile_range_threshold] times in a row, alert
                if under_05_in_a_row == percentile_range_threshold:
                    print("\n==========THIS IS THE ALERT (Consistently under 05th pctl)==========\n")
                    # Send alert to VM
                    if alerts_are_active:
                        send_alert_VM(VM_URL, ts_S1, Consistent_Under_5)


                    under_05_in_a_row = 0 # Reset count once alert occurs

            # Second case: Consistently over 95th pctl
            elif S > p95:
                over_95_in_a_row += 1 # Increase over 95 pctl count
                under_05_in_a_row = 0 # Reset under 5 pctl count

                # If over 95 pctl [percentile_range_threshold] times in a row, alert
                if over_95_in_a_row == percentile_range_threshold:
                    print("\n==========THIS IS THE ALERT (Consistently over 95th pctl)==========\n")
                    # Send alert to VM
                    if alerts_are_active:
                        send_alert_VM(VM_URL, ts_S1, Consistent_Over_95)


                    over_95_in_a_row = 0 # Reset count once alert occurs

            # Last case: Within range
            else:
                # Just reset the count, since nothing out of range
                under_05_in_a_row = 0
                over_95_in_a_row = 0

            # This concludes the comparision for out of range App.Power, but still need
            # to make sure that all this happens within the same time window
            # If time window is about to change, just reset the counts to 0
            # Also do Step 4 checks in same if statement since this happens in the
            # last minute of each time window only

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
            # If final total window sum x factor > 95 pctl, also trigger alert.
            # No need to check total window sum < 5 pctl since Consistent under 5 pctl will trigger

            
            # All of this happens after Step 2 finishes for final minute, which it should have by this point
            # Make sure it's the final mintue
                
                # Check total window sum
                print(f"[TEST] Total window sum: {total_window_sum:.3f}")

                # Set value for the next window lookup table
                value_for_lookup_input = total_window_sum * 0.25 # Total window sum x factor
                print(f"\n[TEST] Value for the next window lookup table: {value_for_lookup_input}")
                print("[TEST] This is also the value compared to the p95 pctl of window")

                # Alert if total window sum > 95 pctl
                if value_for_lookup_input > p95:
                    print("\n==========THIS IS THE ALERT (Total window sum over 95th pctl)==========\n")
                    # Send alert to VM
                    if alerts_are_active:
                        send_alert_VM(VM_URL, ts_S1, Total_Sum_Over_95)


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
        


        # EXPERIMENT 2
        """
        This test case checks whether the voltage coming from the grid simulator is
        over or under the voltage thresholds [under_voltage_threshold], [over_voltage_threshold]
        If so, give an alert
        """
        # Indicate values are for V
        print("##########################")
        print("VOLTAGE")
        print("##########################")
        print()


        dataV1 = query_vm_range(metric_V1, start, end, step)
        dataV2 = query_vm_range(metric_V2, start, end, step)

        if dataV1 and dataV2:
            ts_V1, valV1 = dataV1[-1]
            ts_V2, valV2 = dataV2[-1]

            # Voltage timestamp, should be same as the app.power timestamp, but just in case
            dt_V_str = datetime.datetime.fromtimestamp(ts_V1, pytz.utc).astimezone(pst).strftime("%Y-%m-%d %H:%M:%S %Z")

            # Print the values
            print(f"[TEST] Timestamp: {dt_V_str}, V1: {valV1}, V2: {valV2}")





            # Check for alert now



            # First check V1
            # V1 under voltage [v_range_threshold] times in a row
            if valV1 < under_voltage_threshold:
                under_v1_in_a_row += 1 # Increase under v1 count
                over_v1_in_a_row = 0 # Reset over v1 count

                # If under-voltage [v_range_threshold] times in a row
                if under_v1_in_a_row == v_range_threshold:
                    print("\n==========THIS IS THE ALERT (V1 Under-voltage)==========\n")
                    # Send alert to VM
                    if alerts_are_active:
                        # send here
                        pass

                    under_v1_in_a_row = 0 # Reset under v1 count

            # V1 over voltage [v_range_threshold] times in a row
            elif valV1 > over_voltage_threshold:
                over_v1_in_a_row += 1 # Increase over v1 count
                under_v1_in_a_row = 0 # Reset under v1 count

                # If over-voltage [v_range_threshold] times in a row
                if over_v1_in_a_row == v_range_threshold:   
                    print("\n==========THIS IS THE ALERT (V1 Over-voltage)==========\n")
                    # Send alert to VM
                    if alerts_are_active:
                        pass

                    over_v1_in_a_row = 0 # Reset over v1 count
            # V1 is in range
            else:
                under_v1_in_a_row = 0 # Reset under v1 count
                over_v1_in_a_row = 0 # Reset over v1 count

            # Make sure it stays to one time window
            if time_window_index == current_time_window:
                under_v1_in_a_row = 0 # Reset under v1 count
                over_v1_in_a_row = 0 # Reset over v1 count

            # Finished checking V1


            # Now check V2
            # V2 under voltage [v_range_threshold] times in a row
            if valV2 < under_voltage_threshold:
                under_v2_in_a_row += 1 # Increase under v2 count
                over_v2_in_a_row = 0 # Reset over v2 count

                # If under-voltage [v_range_threshold] times in a row
                if under_v2_in_a_row == v_range_threshold:
                    print("\n==========THIS IS THE ALERT (V2 Under-voltage)==========\n")
                    # Send alert to VM
                    if alerts_are_active:
                        #send here
                        pass

                    under_v2_in_a_row = 0 # Reset under v2 count

                
            # V2 over voltage [v_range_threshold] time in a row
            elif valV2 > over_voltage_threshold:
                over_v2_in_a_row += 1 # Increase over v2 count
                under_v2_in_a_row = 0 # Reset under v2 count

                # If over-voltage [v_range_threshold] times in a row
                if over_v2_in_a_row == v_range_threshold:
                    print("\n==========THIS IS THE ALERT (V2 Over-voltage)==========\n")
                    # Send alert to VM
                    if alerts_are_active:
                        pass

                    over_v2_in_a_row = 0 # Reset over v2 count

            # V2 is in range
            else:
                under_v2_in_a_row = 0 # Reset under v2 count
                over_v2_in_a_row = 0 # Reset over v2 count

            # Make sure it stays to one time window
            if time_window_index == current_time_window:
                under_v2_in_a_row = 0 # Reset under v2 count
                over_v2_in_a_row = 0 # Reset over v2 count


            # Finished checking V2
            
            # At this point V1 and V2 are separately compared to the over and under voltage
            # theshold, and sends an alert if they are outside that range

            # Print v1 and v2 counts
            print("\n[TEST] V1 and V2 counts:")
            print(f"[TEST] Under Threshold: {under_voltage_threshold}, Over Threshold: {over_voltage_threshold}")
            print(f"[TEST] Under v1 count:  {under_v1_in_a_row}")
            print(f"[TEST] Over v1 count: {over_v1_in_a_row}")
            print(f"[TEST] Under v2 count: {under_v2_in_a_row}")
            print(f"[TEST] Over v2 count: {over_v2_in_a_row}\n")

        # Skip everything and print warning if no data is found
        else:
            print("[WARNING] No data received")
            break

        


        # EXPERIMENT 3
        """
        This test case checks whether the frequency of the grid simulator goes outside the
        thresholds [under_frequency], [over_frequency]. If it does, send an alert
        """
        # Indicate values are for F
        print("##########################")
        print("FREQUENCY")
        print("##########################")
        print()


        dataF1 = query_vm_range(metric_F1, start, end, step)
        dataF2 = query_vm_range(metric_F2, start, end, step)

        # If the data exists
        if dataF1 and dataF2:
            ts_F1, valF1 = dataF1[-1]
            ts_F2, valF2 = dataF2[-1]

            # Frequency timestamp, should be same as the app.power timestamp, but just in case
            dt_F_str = datetime.datetime.fromtimestamp(ts_F1, pytz.utc).astimezone(pst).strftime("%Y-%m-%d %H:%M:%S %Z")

            #P Print the values
            print(f"[TEST] Timestamp: {dt_F_str}, F1: {valF1}, F2: {valF2}")

            # Check for alert

            # First check F1
            # F1 under frequency [f_range_threshold] times in a row
            if valF1 < under_frequency_threshold:
                under_f_in_a_row += 1 # Increase under f count
                over_f_in_a_row = 0 # Reset over f count

                # If under-frequency [f_range_threshold] times in a row
                if under_f_in_a_row == f_range_threshold:
                    print("\n==========THIS IS THE ALERT (Under-frequency)==========\n")
                    # Send alert to VM
                    if alerts_are_active:
                        #send here
                        pass

                    under_f_in_a_row = 0 # Reset under f count

            # F1 over frequency [f_range_threshold] time in a row
            elif valF1 > over_frequency_threshold:
                over_f_in_a_row += 1 # Increase over f count
                under_f_in_a_row = 0 # Reset under f count

                # If over-frequency [f_range_threshold] time in a row
                if over_f_in_a_row == f_range_threshold:
                    print("\n==========THIS IS THE ALERT (Over-frequency)==========\n")
                    # Send alert to VM
                    if alerts_are_active:
                        #send here
                        pass

                    over_f_in_a_row = 0 # Reset over f count

            # F1 is in range
            else:
                under_f_in_a_row = 0 # Reset under f count
                over_f_in_a_row = 0 # Reset over f count

            # Make sure it stays to one time window
            if time_window_index == current_time_window:
                under_f_in_a_row = 0 # Reset under f count
                over_f_in_a_row = 0 # Reset over f count

            # Finished checking F1

            # For now, there's no need to check F2, since it's going to be the exact same
            # as F1, so the F1 checks cover both F1 and F2. If needed, just add F2
            # checks below, exact same as F1 check, but with F2 parameters

            # Print f counts
            print("\n[TEST] F counts:")
            print(f"[TEST] Under Threshold: {under_frequency_threshold}, Over Threshold: {over_frequency_threshold}")
            print(f"[TEST] Under f count: {under_f_in_a_row}")
            print(f"[TEST] Over f count: {over_f_in_a_row}\n")
            
        # Skip everything and print warning if no data is found
        else:
            print("[WARNING] No data received")
            break




        # EXPERIMENT 6
        """
        This test case just monitors temperature read by the temperature sensor, and sends
        and alert if the temperature (in F) reaches a value higher than the specified
        threshold [temperature_threshold_F]
        """
        # Indicate values are for Temp
        print("##########################")
        print("TEMPERATURE")
        print("##########################")
        print()


        dataTemp = query_vm_range(metric_Temp, start, end, step)

        if dataTemp:
            ts_Temp, valTemp = dataTemp[-1]

            # Temp timestamp, should be same as the app.power timestamp, but just in case
            dt_Temp_str = datetime.datetime.fromtimestamp(ts_Temp, pytz.utc).astimezone(pst).strftime("%Y-%m-%d %H:%M:%S %Z")

            # Print the values
            print(f"[TEST] Threshold: {temperature_threshold_F} F")
            print(f"[TEST] Timestamp: {dt_Temp_str}, Temperature: {valTemp}\n")

            # Check for alerts
            if valTemp > temperature_threshold_F:
                print("\n==========THIS IS THE ALERT (Excess Temperature)==========\n")
                # Send alert to VM
                if alerts_are_active:
                    send_alert_VM(VM_URL, ts_Temp, High_Temperature)

                                    

        else:
            print("[WARNING] No data received")
            break



        # EXPERIMENT 7: High vibration
        """
        This test case monitors the vibration read by the accelerometer, and sends an alert
        if the vibration is above 20 (default setting) or a custom variable
        [custom_vibration_threshold]
        """
        # Indicate values are for Vib
        print("##########################")
        print("VIBRATION")
        print("##########################")
        print()


        dataVibrationBoolean = query_vm_range(metric_VibBool, start, end, step)
        dataVibrationMagnitude = query_vm_range(metric_VibValue, start, end, step)

        if dataVibrationBoolean and dataVibrationMagnitude:
            
            ts_VibB, valVibB = dataVibrationBoolean[-1]
            ts_VibM, valVibM = dataVibrationMagnitude[-1]

            # Vibration timestamp, should be the same as app.power timestamp, but just in case
            dt_Vib_str = datetime.datetime.fromtimestamp(ts_VibB, pytz.utc).astimezone(pst).strftime("%Y-%m-%d %H:%M:%S %Z")

            # Print the values
            print(f"[TEST] Threshold: {20 if not use_custom_vibration_threshold else custom_vibration_threshold}")
            print(f"[TEST] Timestamp: {dt_Vib_str}, VibrationMag: {valVibM}, VibrationBool: {valVibB}")

            # Check for alert
            if use_custom_vibration_threshold:
                if valVibM > custom_vibration_threshold:
                    print("\n==========THIS IS THE ALERT (Excess Vibration)==========\n")
                    # Send alert to VM
                    if alerts_are_active:
                        send_alert_VM(VM_URL, ts_VibM, High_Vibration)


            else:
                if valVibB:
                    print("\n==========THIS IS THE ALERT (Excess Vibration)==========\n")
                    # Send alert to VM
                    if alerts_are_active:
                        send_alert_VM(VM_URL, ts_VibB, High_Vibration)
                    
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
        
        time.sleep(step if use_live_data else 0.5)
        
