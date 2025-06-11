#!/usr/bin/env python3
import rospy
import requests
from sensor_msgs.msg import BatteryState

class MiRBatteryPublisher:
    def __init__(self):
        rospy.init_node("mir_battery_publisher")

        self.robot_ip = rospy.get_param("~robot_ip", "192.168.12.20")
        self.auth_header = {
            "Authorization": "Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA==",
            "accept": "application/json",
            "Accept-Language": "en_US",
        }

        self.pub = rospy.Publisher("battery_state", BatteryState, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(2.0), self.query_status)

    def query_status(self, event):
        url = f"http://{self.robot_ip}/api/v2.0.0/status"
        try:
            response = requests.get(url, headers=self.auth_header, timeout=1.5)
            response.raise_for_status()
            data = response.json()

            percentage = float(data.get("battery_percentage", -1.0))

            msg = BatteryState()
            msg.percentage = round(percentage / 100.0, 4)  # ROS erwartet 0.0–1.0
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            self.pub.publish(msg)

            rospy.loginfo_throttle(10, f"[{self.robot_ip}] Battery: {percentage:.2f}%")
        except Exception as e:
            rospy.logwarn_throttle(10, f"Battery read failed from {self.robot_ip}: {e}")


if __name__ == "__main__":
    try:
        MiRBatteryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
