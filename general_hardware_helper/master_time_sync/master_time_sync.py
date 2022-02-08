#! /usr/bin/env python3
import sys
import os
from datetime import datetime, timedelta
import requests, json

# ip = "192.168.12.20" # This IP is for executing on the robot
ip = "10.110.130.103" # This IP is for executing on the master computer
host = 'http://' + ip + '/api/v2.0.0/'

headers = {}
headers['Content-Type'] = 'application/json'
headers['Authorization'] = 'Basic RGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='

status_request = requests.get(host + 'status', headers = headers)

header = status_request.headers
robot_time: str = header["Date"]
time: datetime = datetime.strptime(robot_time, "%a, %d %b %Y %H:%M:%S %Z")
new_time = time + timedelta(hours=1)

print("Current time of MiR in GMT: " + str(time))
print("Current time of MiR in MESZ: " + str(new_time))

str_time:str = new_time.strftime("%d %b %Y %H:%M:%S")
hw_str_time:str = new_time.strftime("%Y-%m-%d %H:%M:%S")

rv = os.system('sudo date -s "'+ str_time +'"')
if rv == 0:
   print('Software clock was synced')
else:
   print('Error during software clock time sync')
   
rv = os.system('sudo hwclock --set --date="'+ str_time +'"')
rv2 = os.system('sudo hwclock -s')
if rv == 0 and rv2 == 0:
   print('Hardware clock time was synced')
else:
   print('Error during hardware clock time sync')
