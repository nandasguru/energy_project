import requests
import datetime
import time
import pytz

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
def query_vm_range(metric_name, start_unix, end_unix):
    url = "http://localhost:8428/api/v1/query_range"
    params = {
        "query": metric_name,
        "start": start_unix,
        "end": end_unix,
        "step": 60 # 60s intervals
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

if __name__ == "__main__":

    # Convert desired start/end to Unix timestamps (use your timezone helper!)

    # Use MONTH, DAY, YEAR, HOUR, MINUTE, SECOND
    start = user_to_unix_timestamp(8, 5, 2025, 17, 40, 0)
    end = user_to_unix_timestamp(8, 5, 2025, 18, 0, 0)
    
    metric = "dataESP_V1"
    
    data = query_vm_range(metric, start, end)
    
    if data:
        pst = pytz.timezone('US/Pacific')
        for ts, val in data:
            dt = datetime.datetime.fromtimestamp(ts, pytz.utc).astimezone(pst)
            dt_str = dt.strftime("%Y-%m-%d %H:%M:%S %Z")
                       
            print(f"{dt_str}: {val}")
