#!/usr/bin/env python3
import math
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import WrenchStamped


HELP = r"""
Keyboard wrench setpoint publisher (WrenchStamped)

Publishes to:  ~topic (default: /teleop_test/master_wrench_setpoint)
Frame id:      ~frame_id (default: base_link)

Keys (incremental):
  w / s : +Fx / -Fx
  a / d : +Fy / -Fy
  r / f : +Fz / -Fz

  x     : zero force & torque
  p     : print current setpoint
  + / - : increase / decrease step
  q     : quit

Notes:
  - Each keypress updates the held setpoint and publishes once.
  - Your fake_wrench_hold_publisher will keep republishing the last setpoint at high rate.
"""


class KeyboardWrenchSetpoint:
    def __init__(self):
        self.topic = rospy.get_param("~topic", "/teleop_test/master_wrench_setpoint")
        self.frame_id = rospy.get_param("~frame_id", "base_link")

        self.step_n = float(rospy.get_param("~step_n", 2.0))  # N per keypress
        self.step_scale = float(rospy.get_param("~step_scale", 1.25))
        self.min_step_n = float(rospy.get_param("~min_step_n", 0.1))
        self.max_step_n = float(rospy.get_param("~max_step_n", 50.0))

        self.max_force_norm = float(rospy.get_param("~max_force_norm", 150.0))  # clamp

        self.fx = 0.0
        self.fy = 0.0
        self.fz = 0.0

        self.pub = rospy.Publisher(self.topic, WrenchStamped, queue_size=1)

    def clamp(self):
        n = math.sqrt(self.fx * self.fx + self.fy * self.fy + self.fz * self.fz)
        if n > self.max_force_norm > 0.0:
            s = self.max_force_norm / max(1e-9, n)
            self.fx *= s
            self.fy *= s
            self.fz *= s

    def publish(self):
        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.wrench.force.x = self.fx
        msg.wrench.force.y = self.fy
        msg.wrench.force.z = self.fz
        # keep torques at zero (master controller in this test has use_torques:=false)
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0
        self.pub.publish(msg)

    def print_state(self):
        rospy.loginfo("Setpoint: F=[%.2f %.2f %.2f] N   step=%.2f N   topic=%s",
                      self.fx, self.fy, self.fz, self.step_n, rospy.resolve_name(self.topic))

    @staticmethod
    def get_key(timeout_s: float = 0.1):
        r, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if r:
            return sys.stdin.read(1)
        return ""

    def spin(self):
        rospy.loginfo(HELP.strip("\n"))
        self.print_state()

        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while not rospy.is_shutdown():
                k = self.get_key(0.1)
                if not k:
                    continue

                updated = False
                if k == "w":
                    self.fx += self.step_n
                    updated = True
                elif k == "s":
                    self.fx -= self.step_n
                    updated = True
                elif k == "a":
                    self.fy += self.step_n
                    updated = True
                elif k == "d":
                    self.fy -= self.step_n
                    updated = True
                elif k == "r":
                    self.fz += self.step_n
                    updated = True
                elif k == "f":
                    self.fz -= self.step_n
                    updated = True
                elif k == "x":
                    self.fx = self.fy = self.fz = 0.0
                    updated = True
                elif k == "p":
                    self.print_state()
                elif k == "+" or k == "=":
                    self.step_n = min(self.max_step_n, max(self.min_step_n, self.step_n * self.step_scale))
                    self.print_state()
                elif k == "-":
                    self.step_n = min(self.max_step_n, max(self.min_step_n, self.step_n / self.step_scale))
                    self.print_state()
                elif k == "q":
                    rospy.loginfo("Quit.")
                    break

                if updated:
                    self.clamp()
                    self.publish()
                    self.print_state()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    rospy.init_node("keyboard_wrench_setpoint")
    node = KeyboardWrenchSetpoint()
    node.spin()


if __name__ == "__main__":
    main()

