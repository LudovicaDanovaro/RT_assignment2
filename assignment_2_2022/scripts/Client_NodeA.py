#! /usr/bin/env python

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


