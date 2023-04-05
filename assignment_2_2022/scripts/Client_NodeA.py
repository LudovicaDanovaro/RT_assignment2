#! /usr/bin/env python

""" 
 .. module:: Client_NodeA
    :platform: Unix
    :synopsis: This module contains the code for the client node A. 
    It creates a publisher to publish the custom message to the topic /position_velocity and a subscriber to subscribe to the topic /odom and call the callback function when a message is received. 
    It also calls the action client.

    .. moduleauthor:: *Ludovica Danovaro* S4811864@studenti.unige.it

    Subscriber: 
    /odom

    Publisher: 
    /position_velocity

    Action Client:
    /reaching_goal

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
    It creates the custom message and publishes it to the topic /position_velocity.
    
    Args:
        msg (Odometry): The message received by the subscriber. It contains the position and the linear velocity of the robot.
    
    """
    global pub

    # Get the position and the linear velocity from the message
    position = msg.pose.pose.position
    velocity = msg.twist.twist.linear

    # Create custom message
    pos_vel = Position_Velocity()
    pos_vel.x = position.x
    pos_vel.y = position.y
    pos_vel.vel_x = velocity.x
    pos_vel.vel_y = velocity.y

    # Publish the custom message
    pub.publish(pos_vel)

def action_Client():

    """
    This function creates the action client and sends the goal to the action server. 
    It also waits for the action server to be up and running.

    Args:
        None

    """

    # Create the action client
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction)

    # Wait for the action server to be up and running
    client.wait_for_server()

    while not rospy.is_shutdown():

        #Get the goal from the user
        print("Enter the goal coordinates or type c to cancel it:")
        x = input("x: or c: ")
        y = input("y: or c: ")
        
        if x == "c":
            # Cancel the goal
            client.cancel_goal()
            print("Goal cancelled")
        else:
            # Convert the goal coordinates to float
            x = float(x)
            y = float(y)

            # Create the goal to be sent to the action server
            goal = assignment_2_2022.msg.PlanningGoal()

            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y

            # Send the goal to the action server
            client.send_goal(goal)
            print("Goal sent")


def main():

    """
    This function initializes the ROS node, creates the publisher and the subscriber and calls the action client.

    */odom* is the topic where the robot publishes its position and linear velocity.
    */position_velocity* is the topic where the client node publishes the custom message.

    The position and the linear velocity are passed as a nav_msgs/Odometry message.
    
    """

    global pub

    try:
        # Initialize the rospy node so that the SimpleActionClient can publish and subscribe over ROS.
        rospy.init_node('Client_NodeA')

        # Create a publisher to publish the custom message to the topic /position_velocity
        pub = rospy.Publisher('/position_velocity', Position_Velocity, queue_size=1)

        # Create a subscriber to subscribe to the topic /odom and call the callback function when a message is received
        sub = rospy.Subscriber('/odom', Odometry, callback)

        # Call the action client
        action_Client()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)

if __name__ == '__main__':
    main()


