#!/usr/bin/env python3

# Average the transformation from mir base_link to ur_base_link
# relies on the Qualisys system to be running and publishing the tf

import rospy
import tf
import numpy as np


class UR_calibrate_base_pose:

    def config(self):
        self.ur_base_link = rospy.get_param('~ur_base_link', 'UR10')
        self.mir_base_link = rospy.get_param('~mir_base_link', 'mur620d')
        self.num_samples = rospy.get_param('~num_samples', 100)
        self.ur_default_pose = rospy.get_param('~ur_default_pose', [0.549, -0.318, -0.49, 0.0, 0.0, 0.0])

    def __init__(self):
        rospy.init_node('UR_calibrate_base_pose')
        self.config()
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(20.0)
        self.average_lin = np.zeros(3)
        self.average_rot = np.zeros(4)
        self.current_samples = 0

        # wait for transform to be available
        self.listener.waitForTransform(self.mir_base_link, self.ur_base_link, rospy.Time(0), rospy.Duration(10.0))

        while not rospy.is_shutdown():
            try:
                lin, rot = self.listener.lookupTransform(self.mir_base_link, self.ur_base_link, rospy.Time(0))
                self.average_lin += np.array(lin)
                self.average_rot += np.array(rot)
                if self.current_samples >= self.num_samples:
                    rospy.loginfo('Calibration done')
                    break

                self.current_samples += 1
                self.rate.sleep()
                print('Sample %d' % self.current_samples)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn('Could not get transform')
                continue
        
        self.average_lin /= self.num_samples
        self.average_rot /= self.num_samples

        print('Average lin: %s' % self.average_lin)
        print(self.ur_default_pose[0])
        # substract default pose
        x = self.ur_default_pose[0] - self.average_lin[0]
        y = self.ur_default_pose[1] - self.average_lin[1]
        print(x,y)

        # normalize quaternion
        self.average_rot /= np.linalg.norm(self.average_rot)
        print('Average rot normalized: %s' % self.average_rot)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(self.average_rot)
        self.average_transform = (self.average_lin, yaw)
        print('Average transform:' , self.average_transform)
        #roll, pitch, yaw = tf.transformations.euler_from_quaternion(self.average_rot)
        #self.average_transform = (self.average_lin, roll, pitch, yaw)
        #rospy.set_param('~average_transform', self.average_transform)
        #rospy.loginfo('Num samples: %d' % self.num_samples)







if __name__ == '__main__':
    UR_calibrate_base_pose()
    rospy.spin()