import requests
import time
from datetime import datetime
from datetime import timezone

vmurl = 'http://localhost:8428/write'
memoryLabels = ['MemTotal', 'MemFree', 'MemAvailable']
interval = 10

def read_meminfo():
    meminfo = {}
    with open('/proc/meminfo', 'r') as f:
        for line in f:
            key, value, *_ = line.split()
            meminfo[key.rstrip(':')] = int(value) # value is in kB
    return meminfo

# Test

info = read_meminfo()
for key in memoryLabels:
    print(f"{key}: {info[key]} kB")

print()
print("Insert to VM")
print()

def format_influx(meminfo):
    tags = 'host=pi'
    lines = []
    for key in memoryLabels:
        val_kb = meminfo.get(key, 0)
        line = f"meminfo,{tags},type={key} value={val_kb}"
        lines.append(line)
    return '\n'.join(lines)


# Send to VM

def send_vm(payload):
    try:
        response = requests.post(vmurl, data=payload)
        #print("Payload Sent:")
        #print(payload)
        #print("Response Status Code: ", response.status_code)
        #print("Response Text: ", response.text)
        response.raise_for_status()
        #print("Data Sent")
        
    except Exception as e:
        print("Failed to send: ", e)

def upload():
    meminfo = read_meminfo()
    payload = format_influx(meminfo)
    print(f"Sending:\n{payload}\n")
    send_vm(payload)


def read_and_print():
    query = 'meminfo_value'
    step = '10s'

    endtimeinput = input("Enter first pointt (YYYY-MM-DD HH:MM:SS) - ")
    end_time = int(datetime.strptime(endtimeinput, "%Y-%m-%d %H:%M:%S").timestamp())
    print(end_time)

    starttimeinput = input("How many time steps back: ")
    start_time = end_time - int(starttimeinput * interval)
    print(start_time)

    url = (
        f'http://localhost:8428/api/v1/query_range'
        f'?query={query}'
        f'&start={start_time}'
        f'&end={end_time}'
        f'&step={step}'
    )

    try:
        response = requests.get(url)
        response.raise_for_status()
        data = response.json()

        results = data['data']['result']
        if not results:
            print("No data found.")
        else:
            for result in results:
                mem_type = result['metric'].get('type', 'unknown')
                print(f"\nType: {mem_type}")
                print("Timestamp (UTC)      | Value")
                print("-" * 30)
                for value_pair in result['values']:
                    timestamp = int(value_pair[0])
                    val = value_pair[1]
                    time_str = datetime.fromtimestamp(timestamp, timezone.utc).strftime('%Y-%m-%d %H:%M:%S')
                    print(f"{time_str} | {val}")
    except Exception as e:
        print(f"Error querying VictoriaMetrics: {e}")

def main():
    last = 0
    for i in range(0):
        print()
        print("Iteration: ", i)
        upload()
        time.sleep(interval)
        last += 1
    print()
    print("Iteration: ", last)
    upload()

    time.sleep(3)
    print("Reading data from VM")
    time.sleep(3)

    read_and_print()

if __name__ == '__main__':
    main()