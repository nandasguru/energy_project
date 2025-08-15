import requests
import time

VM_URL = 'http://localhost:8428/write'

def send_alert_vm(url):
    ts_ns = time.time_ns()  # timestamp in nanoseconds
    

    line_protocol = f'Alert,alert=temp test=3 {ts_ns}'

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
