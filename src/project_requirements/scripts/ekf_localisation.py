#!/usr/bin/env python3

import rospy
from project_requirements.msg import SynchronizedMeasurements
from nav_msgs.msg import OccupancyGrid , MapMetaData
import numpy as np
from skimage.draw import line
import tf.transformations as tft
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped



'''
    pose:
      position:
        x: -3.335665307293775e-08
        y: -2.461815733558964e-08
        z: 0.0
      orientation:
        x: -9.396597360899664e-08
        y: 1.273243443821198e-07
        z: -2.777028216836853e-10
        w: 0.9999999999999876

'''


'''
init pose:
      translation:
        x: -3.335665306719148e-08
        y: -2.4618157221729776e-08
        z: -1.882392771634933e-05
      rotation:
        x: -9.396597317448336e-08
        y: 1.2732434436074462e-07
        z: -2.777028220308075e-10
        w: 0.9999999999999875

'''
# queatrnion to euler
x_i = 0.0
y_i = 0.0
z_i = 0.0
w_i = 0.99
euler_init = tft.euler_from_quaternion([x_i, y_i, z_i, w_i])

pub =None
u_estimate=np.array([0,0,0])
cov_estimate=np.array([[0,0,0],[0,0,0],[0,0,0]])

x_init=0
y_init=0
z_init=0
theta_init=0

last_time_step=0

#alphas = [.2 .03 .09 .08 0 0]; % robot-dependent motion noise parameters

alpha1=0.2
alpha2=0.03
alpha3=0.09
alpha4=0.08

alpha_r =0.08
alpha_theta=0.08
alpha_s=0.05


def prediction_step(data):
    #print('now prediction step')
    global u_estimate, cov_estimate,last_time_step, alpha1, alpha2, alpha3, alpha4 , euler_init, pub
    delta_t=rospy.Time.now().to_sec()-last_time_step
    last_time_step=rospy.Time.now().to_sec()


    v=data.linear.x
    w=data.angular.z
    u_estimate= u_estimate + np.array([
                                v*delta_t*np.cos(u_estimate[2] + (w*delta_t)/2),
                                v*delta_t*np.sin(u_estimate[2] + (w*delta_t)/2),
                                w*delta_t
                                ])
    G = np.array([
                [1, 0, -v*delta_t*np.sin(u_estimate[2] + (w*delta_t)/2)],
                [0, 1, v*delta_t*np.cos(u_estimate[2] + (w*delta_t)/2)],
                [0, 0, 1]
                ])  # Jacobian of motion model
    V = np.array([
                [np.cos(u_estimate[2] + (w*delta_t)/2), -0.5*np.sin(u_estimate[2] + (w*delta_t)/2)],
                [np.sin(u_estimate[2] + (w*delta_t)/2), 0.5*np.cos(u_estimate[2] + (w*delta_t)/2)],
                [0, 1]
                ])
    M = np.array([
                [alpha1*(v**2) + alpha2*(w**2), 0],
                [0, alpha3*(v**2) + alpha4*(w**2)]
                ])

    cov_estimate = np.dot(np.dot(G, cov_estimate), G.T) + np.dot(np.dot(V, M), V.T)


    # message_to_send=PoseStamped()
    # message_to_send.pose.position.x=u_estimate[0]
    # message_to_send.pose.position.y=u_estimate[1]
    # message_to_send.pose.position.z=z_init


    # q = tft.quaternion_from_euler(euler_init[0], euler_init[1], u_estimate[2])
    # message_to_send.pose.orientation.x=q[0]
    # message_to_send.pose.orientation.y=q[1]
    # message_to_send.pose.orientation.z=q[2]
    # message_to_send.pose.orientation.w=q[3]

    # # set frame name
    # message_to_send.header.frame_id = 'robot_map'




    # pub.publish(message_to_send)











def correction_step(data):
    #print('now correction step')
    pos_x = data.odom.pose.pose.position.x
    pos_y = data.odom.pose.pose.position.y
    orientation = [data.odom.pose.pose.orientation.x, data.odom.pose.pose.orientation.y, data.odom.pose.pose.orientation.z, data.odom.pose.pose.orientation.w]
    euler = tft.euler_from_quaternion(orientation)
    data = np.array([pos_x, pos_y, euler[2]])

    global u_estimate, cov_estimate,last_time_step, alpha1, alpha2, alpha3, alpha4 , euler_init, pub, alpha_r, alpha_theta, alpha_s

    Q=np.array([
                [alpha_r, 0, 0],
                [0, alpha_theta, 0],
                [0, 0, alpha_s]
                ])
    # C is identity 3x3
    C=np.array([
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]
                ])
    K= np.dot(np.dot(cov_estimate, C.T), np.linalg.inv(np.dot(np.dot(C, cov_estimate), C.T) + Q))
    u_estimate= u_estimate + np.dot(K, data - np.dot(C, u_estimate))
    cov_estimate = np.dot((np.eye(3) - np.dot(K, C)), cov_estimate)



    message_to_send=PoseStamped()
    message_to_send.pose.position.x=u_estimate[0]
    message_to_send.pose.position.y=u_estimate[1]
    message_to_send.pose.position.z=z_init


    q = tft.quaternion_from_euler(euler_init[0], euler_init[1], u_estimate[2])
    message_to_send.pose.orientation.x=q[0]
    message_to_send.pose.orientation.y=q[1]
    message_to_send.pose.orientation.z=q[2]
    message_to_send.pose.orientation.w=q[3]

    # set frame name
    message_to_send.header.frame_id = 'robot_map'
    #time stamp
    message_to_send.header.stamp = rospy.Time.now()




    pub.publish(message_to_send)







def listener():
    global pub
    rospy.init_node('ekf_localisation', anonymous=True)
    pub = rospy.Publisher('/ekf_localisation', PoseStamped, queue_size=10)


    rospy.Subscriber("/robot/cmd_vel", Twist, prediction_step)
    rospy.Subscriber("/synchronized_data", SynchronizedMeasurements, correction_step)



    rospy.spin()


if __name__ == '__main__':
    listener()

