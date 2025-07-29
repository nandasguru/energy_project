import requests
import time
from time import sleep

"""
#ts = int(time.time())

line = 'test_metric{{device="pi"}} 1'

r = requests.post('http://localhost:8428/api/v1/import/prometheus', data=line)
print("Send status: ", r.status_code)

sleep(5)


q = requests.get('http://localhost:8428/api/v1/query?query=test_metric{device="pi"}')
print("Query: ", q.json())
"""

line = 'test_metric{device="pi"} 1'
send_resp = requests.post('http://localhost:8428/api/v1/import/prometheus', data=line)
print("Send status:", send_resp.status_code, send_resp.text)

sleep(10)

query_resp = requests.get('http://localhost:8428/api/v1/query?query=test_metric{device="pi"}')
print("Query result:", query_resp.json())

names_resp = requests.get('http://localhost:8428/api/v1/label/__name__/values')
print("Available metric names:", names_resp.json())