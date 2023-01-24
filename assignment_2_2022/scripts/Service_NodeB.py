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
from assignment_2_2022.srv import Goal, GoalResponse

# Initialization of the variables that will be used to store the number of goals cancelled and reached
num_cancelled = 0;
num_reached = 0;

def callback(msg):
    
    global num_cancelled, num_reached

    # Get the status of the result from the message
    status = msg.status.status

    #If status is 2, the goal is cancelled
    if status == 2:
        # If the goal has been cancelled, increment the number of goals cancelled
        num_cancelled += 1
        print("Goal cancelled")
    
    #If status is 3, the goal is reached
    elif status == 3:
         # If the goal has been reached, increment the number of goals reached
        num_reached += 1
        print("Goal reached")


def update_goals(req):

    return GoalResponse(num_cancelled, num_reached)


def main():

    # Initialize the node
    rospy.init_node('Service_NodeB_server')

    # Create the service /update_goals
    service = rospy.Service('/update_goals', Goal, update_goals)

    # Create the subscriber 
    sub = rospy.Subscriber('/reaching_goal/result', assignment_2_2022.msg.PlanningActionResult, callback)

    # Wait for the service to be called
    rospy.spin()

if __name__ == '__main__':
    main()
