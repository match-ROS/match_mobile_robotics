#! /usr/bin/env python3
import sys
import os
from datetime import datetime, timedelta
import requests, json

ip = "192.168.12.20" # This IP is for executing on the robot
# ip = "10.110.130.23" # This IP is for executing on the master computer
host = 'http://' + ip + '/api/v2.0.0/'

headers = {}
headers['Content-Type'] = 'application/json'
headers['Authorization'] = 'Basic RGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='

status_request = requests.get(host + 'status', headers = headers)

header = status_request.headers
robot_time: str = header["Date"]
time: datetime = datetime.strptime(robot_time, "%a, %d %b %Y %H:%M:%S %Z")
new_time = time + timedelta(hours=2)

print("Current time of MiR in GMT: " + str(time))
print("Current time of MiR in MESZ: " + str(new_time))

str_time:str = new_time.strftime("%d %b %Y %H:%M:%S")

rv = os.system('sudo date -s "'+ str_time +'"')
if rv == 0:
   print('Time was synced')
else:
   print('Error during time sync')
