#!/usr/bin/env python3

import rospy
from project_requirements.msg import SynchronizedMeasurements
# import math
# import message_filters
# import tf.transformations as tft
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import LaserScan
# from std_msgs.msg import Header
# import numpy as np


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print(data)

def listener():

        rospy.init_node('listener_sync')

        rospy.Subscriber("/synchronized_data", SynchronizedMeasurements, callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    listener()

