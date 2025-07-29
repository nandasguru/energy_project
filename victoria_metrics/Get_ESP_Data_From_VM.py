from datetime import timezone, timedelta
import requests
import datetime
import time

URL_single = 'http://localhost:8428/api/v1/query'
URL_range = 'http://localhost:8428/api/v1/query_range'
pst = timezone(timedelta(hours=-7))

# Query for the latest metric
def query_latest_metric(metric_name: str):
    try:
        # Send GET request with metric query
        params = {'query': metric_name}
        response = requests.get(URL_single, params=params)

        # Raise error if HTTP request failed
        response.raise_for_status()
        result_json = response.json()

        # Check if query succeeded
        if result_json['status'] != 'success':
            print(f"Query failed: {result_json.get('error', 'Unknown error')}")
            return
        
        # Results array
        results = result_json['data']['result']

        if not results:
            print(f"No data found for metric '{metric_name}'.")
            return
        
        # Get latest value
        latest = results[0]['value']
        timestamp_unix = float(latest[0])
        value = latest[1]

        # Convert to readable format
        # UTC to PST
        

        timestamp_human = datetime.datetime.fromtimestamp(timestamp_unix, tz=pst).strftime('%Y-%m-%d %H:%M:%S')
        
        print(f"Metric: {metric_name}")
        print(f"Timestamp: {timestamp_human} (Unix: {timestamp_unix})")
        print(f"Value: {value}")

    except requests.RequestException as e:
        print(f"HTTP request error {e}")

    except(KeyError, IndexError, ValueError) as e:
        print (f"Error parsing response: {e}")









def query_range_metric(metric_name: str, steps: int, end_time: int = None, step_seconds: int = 10):

    if end_time is None:
        end_time = int(time.time())            # current time in seconds (Unix) if non specified

    print("End time is: ", end_time)

    duration_seconds = (steps - 1) * step_seconds # Account for smaller time step
    start_time = end_time - duration_seconds

    print(f"Querying '{metric_name}' from {datetime.datetime.fromtimestamp(start_time)} to {datetime.datetime.fromtimestamp(end_time)}")
    print(f"Step interval: {step_seconds} seconds, Steps: {steps}")

    params = {
        'query': metric_name,
        'start': start_time,
        'end': end_time,
        'step': step_seconds,
    }

    try:
        response = requests.get(URL_range, params=params)
        response.raise_for_status()
        data = response.json()

        if data['status'] != 'success':
            print(f"Query failed: {data.get('error', 'Unknown error')}")
            return

        results = data['data']['result']
        if not results:
            print(f"No data found for metric '{metric_name}' in the given time range.")
            return

        print(f"Data for metric '{metric_name}' from {datetime.datetime.fromtimestamp(start_time, tz=pst)} to {datetime.datetime.fromtimestamp(end_time, tz=pst)}:")
        for result in results:
            print(f"\nMetric labels: {result['metric']}")
            for timestamp_str, value_str in result['values']:
                ts = float(timestamp_str)
                dt = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
                print(f"{dt} (Unix {ts}): {value_str}")

    except requests.RequestException as e:
        print(f"HTTP request error: {e}")

    except (KeyError, IndexError, ValueError) as e:
        print(f"Error parsing response: {e}")
        

if __name__ == "__main__":
    v1 = "dataESP_V1"
    v2 = "dataESP_V2"
    i1 = "dataESP_I1"
    i2 = "dataESP_I2"
    p1 = "dataESP_P1"
    p2 = "dataESP_P2"
    s1 = "dataESP_S1"
    s2 = "dataESP_S2"
    f1 = "dataESP_F1"
    f2 = "dataESP_F2"
    ph1 = "dataESP_PH1"
    ph2 = "dataESP_PH2"
    
    list_of_metrics = [v1, v2, i1, i2, p1, p2, s1, s2, f1, f2, ph1, ph2]
    list_of_metrics_original_order = [v1, i1, p1, s1, f1, ph1, v2, i2, p2, s2, f2, ph2]
    for metric in list_of_metrics_original_order:
        print("----------")
        query_latest_metric(metric)
        

    #query_latest_metric(v1)

    #query_range_metric(metric_to_query, steps = 10, end_time = 1752541200) 