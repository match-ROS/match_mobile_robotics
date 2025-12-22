#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int32
def increment_path_index():
    # Initialize the ROS node
    rospy.init_node('increment_path_index', anonymous=True)

    path_index_topic = rospy.get_param('~path_index_topic', '/path_index')
    initial_path_index = rospy.get_param('~initial_path_index', 0)

    # Create a publisher to the 'path_index' topic
    pub = rospy.Publisher(path_index_topic, Int32, queue_size=10)

    # Set the rate at which to publish messages
    rate = rospy.Rate(10.0)  # 3.33 Hz

    # Initialize the path index
    path_index = initial_path_index

    while not rospy.is_shutdown():
        # Increment the path index
        path_index += 1

        # Create a message with the current path index
        msg = Int32()
        msg.data = path_index

        # Publish the message
        pub.publish(msg)

        # Log the published path index
        #rospy.loginfo(f"Published path index: {path_index}")

        # Sleep for the specified rate
        rate.sleep()


if __name__ == '__main__':
    try:
        increment_path_index()
    except rospy.ROSInterruptException:
        pass