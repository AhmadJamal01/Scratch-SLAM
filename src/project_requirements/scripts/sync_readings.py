#!/usr/bin/env python3

import rospy
import math
import message_filters
import tf.transformations as tft
from project_requirements.msg import SynchronizedMeasurements
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np


def callback(odom, laser):
    global publisher
    sync_msg = SynchronizedMeasurements()
    sync_msg.header = laser.header
    sync_msg.odom = odom
    sync_msg.laser = laser
    publisher.publish(sync_msg)



def main():
    rospy.init_node('sync_robot_measurements')
    odom_sub = message_filters.Subscriber(
        '/robot/robotnik_base_control/odom', Odometry, queue_size=1)
    laser_sub = message_filters.Subscriber(
        '/scan_multi', LaserScan, queue_size=1)
    ts = message_filters.ApproximateTimeSynchronizer(
        [
            odom_sub,
            laser_sub,
        ],
        1,
        0.01,
    )
    ts.registerCallback(callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        publisher = rospy.Publisher(
            'synchronized_data', SynchronizedMeasurements, queue_size=1)
        main()
    except rospy.ROSInterruptException:
        pass
