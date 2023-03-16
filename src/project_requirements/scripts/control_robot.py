#!/usr/bin/env python3

# create a ROS node for controlling the robot using input from the keyboard
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys
import select
import tty
import termios


def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def control_robot():
    # initialize the node
    rospy.init_node('control_robot', anonymous=True)
    # create a publisher to publish the velocity commands
    pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)
    # create a rate object to control the frequency of publishing
    rate = rospy.Rate(10)  # 10hz
    # create a Twist object to store the velocity commands
    vel = Twist()
    liner_x = 0.0
    angular_z = 0.0
    c = ''
    old_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())
        while 1:

            if isData():
                c = sys.stdin.read(1)
                if c == 'w':
                    liner_x += 0.5
                elif c == 's':
                    liner_x += -0.5
                elif c == 'd':
                    angular_z += -0.5
                elif c == 'a':
                    angular_z += 0.5
                elif c == 'q':
                    liner_x = 0.0
                    angular_z = 0.0
            else:
                liner_x *= 0.5
                angular_z *= 0.5

            vel.linear.x = liner_x
            vel.angular.z = angular_z
            # publish the velocity commands
            # print(vel)
            pub.publish(vel)
            # sleep for the remaining time to maintain the desired rate
            # flush the input buffer
            sys.stdin.flush()
            rate.sleep()

            if c == 'q':
                break

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == '__main__':
    try:
        control_robot()
    except rospy.ROSInterruptException:
        pass
