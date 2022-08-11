#!/usr/bin/env python3
from subprocess import Popen, PIPE
from os.path import abspath, dirname, join
import rospy

rospy.init_node('clock_server')
server_ip = rospy.get_param('~server_ip', 'localhost')
server_port = rospy.get_param('~server_port', '4567')

rospy.loginfo("running clock server on: " + server_ip + " port: " + str(server_port))

path = abspath(join(dirname(__file__), 'ckserver'))
output_string = Popen([path,server_ip, str(server_port)], stdout=PIPE, stderr=PIPE).communicate()



