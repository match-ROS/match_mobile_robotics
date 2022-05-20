#! /usr/bin/env python3
import sys
import os
from datetime import datetime, timedelta
from time import sleep
import time
import requests, json

ip = "192.168.12.20" # This IP is for executing on the robot
# ip = "10.145.8.35"    #This IP is for executing on the master computer
host = 'http://' + ip + '/api/v2.0.0/'

headers = {}
headers['Content-Type'] = 'application/json'
headers['Accept-Language'] = 'en_US'
headers['Authorization'] = 'Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='

# time_new = datetime.now() + timedelta(hours=2)
time_new = datetime.now()

data = {
   "datetime": f"{time_new.isoformat(timespec='milliseconds')}Z"
   # "datetime": "2022-05-16T13:51:21.479Z" 
}
status_request = requests.put(host + 'status', headers = headers, data=json.dumps(data))

# print(f"status_request Headers: {status_request.headers}")


