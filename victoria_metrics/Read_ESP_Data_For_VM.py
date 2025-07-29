import requests
import serial
import time
import json
import re

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
URL = 'http://localhost:8428/write'

# -----------------------------------------------------------

def convertToKeyValuePairs(line):

    # print("Raw line: ", line) - debug

    # Remove surrounding braces
    line = line.strip()
    if line.startswith("{") and line.endswith("}"):
        line = line[1:-1]

    # print("Remove braces: ", line) - debug

    # Fix missing commas
    line = re.sub(r'(\d)\s+([A-Za-z])', r'\1, \2', line)
    # print("Fix missing commmas: ", line) - debug

    # Add quotes around keys
    line = re.sub(r'([^,"\s]+)\s*:', r'"\1":', line)
    # print("added quotes: ", line) - debug

    # Put braces back for JSON
    json_line = "{" + line + "}"
    # print("json_line: ", repr(json_line)) - debug

    try:
        return json.loads(json_line)
    except json.JSONDecodeError as e:
        print("JSON parse error: ", e)
        return {}
    
# Format the parsed data into a Victoria Metric ready format    
def formatForVM(data: dict, measurement_name: str) -> str:
    fields = []

    for key, value in data.items():
        # only include keys with int or float values
        if isinstance(value, (int, float)):
            fields.append(f"{key}={value}")
        else:
            # skip non-numeric values
            pass
    
    # Join key value pairs with commas
    fields_str = ",".join(fields)

    # Current timestamp in nanoseconds
    timestamp_ns = time.time_ns()
    print("Current time in nanoseconds: ", timestamp_ns)

    # Final formatted string
    line = f"{measurement_name} {fields_str} {timestamp_ns}"

    return line

# Send the formatted data to Victoria Metrics
def sendToVM(protocol: str) -> bool:

    try:
        response = requests.post(URL, data=protocol)

        if response.status_code == 204:
            print("Data sent successfully to Victoria Metrics.")
            return True
        else:
            print(f"Failed to send to Victoria Metrics")
            return False
        
    except requests.RequestException as e:
        print(f"Error sending data to Victoria Metrics: {e}")
        return False

# -----------------------------------------------------------

def readFromESP32AndSendToVM():
    
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud...\n")

    try:
        while True:
            
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                print("--------------------START--------------------")
                print("Raw line: \n", line)
                print()
                parsed_line = convertToKeyValuePairs(line)
                print("Parsed data: \n", parsed_line)
                print()
                vm_line = formatForVM(parsed_line, "dataESP")
                print("\nFinal formatted string for VM:\n", vm_line)
                print()

                sendToVM(vm_line)

                print("--------------------END--------------------")
            
            
    except KeyboardInterrupt:
        print("\nStopped reading serial.")
        ser.close()

# Test Method
readFromESP32AndSendToVM()

