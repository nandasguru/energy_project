import requests
import time
import datetime

VM_URL = 'http://localhost:8428/write'

def send_alert_vm(url):
    ts_ns = time.time_ns()  # timestamp in nanoseconds
    print(ts_ns)

    dt = datetime.datetime(2025, 8, 14, 12, 45, 0)

    ts_before = int((dt.timestamp())- 1)
    ts_real = int(dt.timestamp())
    ts_after = int((dt.timestamp()) + 1)

    

    line_protocol = (
        f'Alert test=0 {ts_before}\n'
        f'Alert test=2 {ts_real}\n'
        f'Alert test=0 {ts_after}'
    )

    print(line_protocol)

    try:
        response = requests.post(url, data=line_protocol)
        if response.status_code == 204:
           print("[TEST] Msg sent successfully!")
        else:
            print(f"Failed. Status Code: {response.status_code}")
    except Exception as e:
        print(f"Error sending test alert: {e}")

send_alert_vm(VM_URL)
send_alert_vm(VM_URL)

