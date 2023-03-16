#!/usr/bin/env python3

import rospy
from project_requirements.msg import SynchronizedMeasurements
from nav_msgs.msg import OccupancyGrid , MapMetaData
import numpy as np
from skimage.draw import line
import tf.transformations as tft


# import math
# import message_filters
# import tf.transformations as tft
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import LaserScan
# from std_msgs.msg import Header
# import numpy as np

pub =None
occupancy_grid = None
map_meta_data = None







def create_transform_matrix(translation, rotation):
    translation_matrix= tft.translation_matrix(translation)
    rotation_matrix= tft.quaternion_matrix(rotation)
    return translation_matrix@rotation_matrix

def xy_to_ij(x, y):
    global map_meta_data
    i = (y - map_meta_data.origin.position.y) / map_meta_data.resolution
    j = (x - map_meta_data.origin.position.x) / map_meta_data.resolution
    i = np.array(i, dtype=int)
    j = np.array(j, dtype=int)
    return i, j


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print('update_map ya regala')
    global occupancy_grid, map_meta_data, pub

    # angles= np.array(data.laser.angle_min + np.arange(len([data.laser.ranges])) * data.laser.angle_increment)
    angles = np.linspace(data.laser.angle_min, data.laser.angle_max, len(data.laser.ranges))

    pos_x = data.odom.pose.pose.position.x
    pos_y = data.odom.pose.pose.position.y
    orientation = [data.odom.pose.pose.orientation.x, data.odom.pose.pose.orientation.y, data.odom.pose.pose.orientation.z, data.odom.pose.pose.orientation.w]
    transform_matrix = create_transform_matrix([pos_x, pos_y, 0], orientation)

    x = np.array(data.laser.ranges) * np.cos(angles)
    y = np.array(data.laser.ranges) * np.sin(angles)


    laser_points = np.array([x, y, np.zeros(len(x)), np.ones(len(x))])
    laser_points = transform_matrix @ laser_points
    x = laser_points[0, :]
    y = laser_points[1, :]

    x, y = xy_to_ij(x, y)
    pos_x, pos_y = xy_to_ij(pos_x, pos_y)


    for i in range(x.size):

        if x[i] < 0 or x[i] >= map_meta_data.width or y[i] < 0 or y[i] >= map_meta_data.height:
            continue
        # if out of range contune

        if data.laser.ranges[i] >= data.laser.range_max or data.laser.ranges[i] <= data.laser.range_min:
            continue


        rr, cc = line(pos_x,pos_y, x[i], y[i])
        rr, cc = rr[:-1], cc[:-1]

        empty_inices= np.ravel_multi_index((rr, cc), (map_meta_data.width, map_meta_data.height))
        empty_inices = empty_inices.astype(int)
        occupancy_grid.data[empty_inices] = 0


        hitpoint_idx = np.ravel_multi_index((x[i], y[i]), (map_meta_data.width, map_meta_data.height))
        occupancy_grid.data[hitpoint_idx] = 100

        # occupancy_grid.data[y[i] * map_meta_data.width + x[i]] = 100

    occupancy_grid.header.stamp = data.header.stamp
    pub.publish(occupancy_grid)



def listener():

        global occupancy_grid, map_meta_data, pub

        # create publisher
        pub = rospy.Publisher('map_req3', OccupancyGrid, queue_size=1)

        rospy.init_node('listener_sync')


        map_meta_data = MapMetaData()
        map_meta_data.resolution = 0.02
        map_meta_data.width = 4992
        map_meta_data.height = 4992
        map_meta_data.origin.position.x = -50
        map_meta_data.origin.position.y = -50
        map_meta_data.origin.position.z = 0

        map_meta_data.origin.orientation.x = 0
        map_meta_data.origin.orientation.y = 0
        map_meta_data.origin.orientation.z = 0
        map_meta_data.origin.orientation.w = 1


        occupancy_grid = OccupancyGrid()
        occupancy_grid.info = map_meta_data
        occupancy_grid.header.frame_id = "robot_map"
        occupancy_grid.data = np.ones(map_meta_data.width * map_meta_data.height, dtype=np.int8) * -1

        rospy.Subscriber("/synchronized_data", SynchronizedMeasurements, callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    listener()

