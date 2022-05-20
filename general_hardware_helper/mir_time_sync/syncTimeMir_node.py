#! /usr/bin/env python3
from datetime import datetime, timedelta
import requests, json
import rospy
import std_msgs.msg as std_msg


class TimeSync(object):

   def __init__(self, ip=None):
      """Class for syncing time of Mir with time of the PC. Subscribes to topic /syncTime

      Args:
          ip (str, optional): ip of MiR for API access. If None: ip="192.168.12.20"
      """
      if ip is None:
         self.ip = "192.168.12.20"  # This IP is for executing on the robot
         # self.ip = "10.145.8.35" # This IP is for executing on the master computer
      else:
         self.ip = ip

      self.host = 'http://' + self.ip + '/api/v2.0.0/'
      
      self.headers = {}
      self.headers['Content-Type'] = 'application/json'
      self.headers['Accept-Language'] = 'en_US'
      self.headers['Authorization'] = 'Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='

      self.sub = rospy.Subscriber("/syncTime", std_msg.Bool, self.sync_time)

   def sync_time(self, sync=std_msg.Bool()):
      """function to sync time. Called by subscriber /syncTime

      Args:
          sync (bool): sync if true

      Returns:
          bool: success (not implemented)
      """
      if not sync.data:
         return False

      # time_new = datetime.now() + timedelta(hours=2)
      time_new = datetime.now()

      data = {
         "datetime": f"{time_new.isoformat(timespec='milliseconds')}Z"
         # "datetime": "2022-05-16T13:51:21.479Z" 
      }
      status_request = requests.put(self.host + 'status', headers = self.headers, data=json.dumps(data))

      return True


if __name__ == "__main__":
    rospy.init_node("Time_Synchronization")
    t_sync = TimeSync()
    rospy.spin()

