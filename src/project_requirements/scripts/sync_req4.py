#!/usr/bin/env python3

import rospy
import math
import message_filters
import tf.transformations as tft
from project_requirements.msg import SynchronizedMeasurements
from project_requirements.msg import Synch_message_req4

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

import numpy as np
publisher=None

def callback(pose, laser):
    global publisher
    sync_msg = Synch_message_req4()
    sync_msg.header = laser.header
    sync_msg.pose = pose
    sync_msg.laser = laser
    publisher.publish(sync_msg)



def main():
    rospy.init_node('sync_req4')
    pose_sub = message_filters.Subscriber(
        '/ekf_localisation', PoseStamped, queue_size=1)
    laser_sub = message_filters.Subscriber(
        '/scan_multi', LaserScan, queue_size=1)
    ts = message_filters.ApproximateTimeSynchronizer(
        [
            pose_sub,
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
            'synch_req4', Synch_message_req4, queue_size=1)
        main()
    except rospy.ROSInterruptException:
        pass
