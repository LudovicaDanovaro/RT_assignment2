#! /usr/bin/env python

""" 
 .. module:: Service_NodeB
    :platform: Unix
    :synopsis: This module creates a service that returns the number of goals cancelled and reached. 
    It also subscribes to the topic */reaching_goal/result* to get the status of the result and increment the number of goals cancelled and reached.

    .. moduleauthor:: *Ludovica Danovaro* S4811864@studenti.unige.it

    Subscriber: 
    /reaching_goal/result

    Service:
    /update_goals

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
from assignment_2_2022.srv import Goal, GoalResponse

# Initialization of the variables that will be used to store the number of goals cancelled and reached
num_cancelled = 0;
num_reached = 0;

def callback(msg):

    """
    This function is called when a message is received by the subscriber. 
    It increments the number of goals cancelled and reached depending on the status of the result.

    Args:
        msg (PlanningActionResult): The message received by the subscriber. It contains the status of the result.

    """
    
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

    """
    This function is called when the service is called. It returns the number of goals cancelled and reached.
    """

    return GoalResponse(num_cancelled, num_reached)


def main():

    """
    This function initializes the ROS node and creates the service and the subscriber.
    */reaching_goal/result* is the topic where the action server publishes the status of the result.
    */update_goals* is the service that returns the number of goals cancelled and reached.
    
    """
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
