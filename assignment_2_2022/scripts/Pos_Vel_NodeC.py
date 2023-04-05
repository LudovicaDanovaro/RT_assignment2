#! /usr/bin/env python

""" 
 .. module:: Pos_Vel_NodeC
    :platform: Unix
    :synopsis: This module is the third node of the assignment 2. It subscribes to the topic /position_velocity and prints the distance from the goal and the average velocity.

    .. moduleauthor:: *Ludovica Danovaro* S4811864@studenti.unige.it

    Subscriber: 
    /position_velocity

"""

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import assignment_2_2022.msg
from tf import transformations
from std_srvs.srv import *
import time
import sys
import select
from assignment_2_2022.msg import Position_Velocity


def callback(msg):

    """
    This function is called when a message is received by the subscriber.
    It prints the distance from the goal and the average velocity.

    """

    # Get the desired position
    des_x = rospy.get_param("des_pos_x")
    des_y = rospy.get_param("des_pos_y")

    # Get the actual position and the linear velocity from the message
    x = msg.x
    y = msg.y
    v_x = msg.vel_x
    v_y = msg.vel_y

    # Compute the distance between the actual position and the desired position
    distance = math.dist([des_x, des_y], [x, y])

    # Compute the average velocity
    avg_vel = math.sqrt(v_x**2 + v_y**2)

    # Set frequency
    freq = rospy.get_param("/set_frequency")
    rate = rospy.Rate(freq)

    # Print the distance and the average velocity
    print("Distance from the goal:", distance)
    print("Average velocity:", avg_vel)
    print()

    rate.sleep()


def main():

    """
    This function initializes the ROS node and creates the subscriber.
    */position_velocity* is the topic where the client node publishes the custom message.

    The position and the linear velocity are passed as a nav_msgs/Odometry message.    

    """

    # Initialize the node
    rospy.init_node('Pos_Vel_NodeC')

    # Create the subscriber to the topic /position_velocity
    sub = rospy.Subscriber('/position_velocity', Position_Velocity, callback)

    # Wait for the service to be called
    rospy.spin()

if __name__ == '__main__':
    main()
