# Research Track 1 - Assignment 2

Ludovica Danovaro - S4811864

# Introduction
In this assignment, we are requested to implement three nodes for controlling a mobile robot in a 3D simulation environment called *Gazebo*. 
So I developed, by using **ROS**, three nodes and the launch file:
- (A) A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the topic /odom. 
- (B) A service node that, when called, prints the number of goals reached and cancelled;
- (C) A node that subscribes to the robot’s position and velocity (using the custom message) and prints the distance of the robot from the target and the robot’s average speed. Use a parameter to set how fast the node publishes the information.
-  Create a launch file to start the whole simulation. Set the value for the frequency with which node (C) publishes
the information.


# Nodes
There are six nodes in the package:

- `bug_as.py` is the action server node that gets the desired position from the client and calls the needed services to bring the robot to the desired position, setting the position as a ROS parameter
- `go_to_point_service.py` is the service node that, when called, makes the robot move toward the desired position, retrieved from the ROS parameter
- `wall_follow_service.py` is the service node that, when called, makes the robot move around an obstacle (a wall, an object...)
- `Client_NodeA.py` is the action client nodethat allows the user to set the desired position or to cancel the desired position and it publishes the robot position and velocity as a custom message on the /*position_velocity* topic, relying on the values of the /*odom* topic.
- `Service_NodeB.py` is the service node that, when called, prints the number of times a goal has been reached and the number of times a goal has been canceled
-  `Pos_Vel_NodeC.py` is the node that subscribes to the robot’s position and velocity from the */position_velocity* topic as a custom message and prints the distance of the robot from the target and the robot’s average speed with a frequency setted as a parameter in the lauch file

![rosgraph](https://user-images.githubusercontent.com/107572039/211221770-f5538c04-3884-4336-9099-b9ee421dbd1a.png)



After the program has started, you can interact with four windows:

- **Rviz** is a ROS visualization 
- **Gazebo** is the 3D simulator environment with the obstacles and the robot 
- **ActionClient_nodeA.py** is the window where the user can set the goal position or cancel it from keyboard
- **Dist_Speed_nodeC.py** is the window where distance from target and average speed of the robot are showed

- To know the number of goals reached and canceled, type on another tab:

      rosservice call /update_goals


- You can set the frequency from the launch file `assignment1.launch` inside of the `launch` folder. To do so, modify the value in the following line:

```xml
    <!--Parameter to set the frequency the info is printed with-->
    <param name="publish_frequency" type="double" value="1.0" />
```

and relaunch the program.


# Installing and Running
-  First of all, you have to install **xterm**:

       sudo apt install xterm

- To install the module, you need to go inside the `src` folder of your ROS workspace and run the following command:

    git clone https://github.com/LudovicaDanovaro/RT_assignment2.git
    

and from the root directory of your ROS workspace run the command:

    catkin_make

- now run the master by typing:

      roscore 

- To run the program, you need to have installed in your system the program **xterm**. To install it, run:

    sudo apt-get install xterm

- to run the code, type the following command:

      roslaunch assignment_2_2022 assignment1.launch



# Improvment
The program has some flaws and in a future update, it would be possible to fix them and improve the code. The problems are:
- If the desired position is exactly where an obstacle is, the robot will bump into the obstacle and sometimes flip over, trying to reach it. To avoid this behavior, the robot, when approaching the position, could recognize that the position is unreachable, stop in front of the obstacle and inform the user.
- If the robot encounters an obstacle while moving to the desired position, it will go around it always in a clockwise way, sometimes getting further from the goal. To fix this, the robot could choose in which way it should go around the obstacle by choosing the one that will make him go closer to the desired position.
- It's not clear where the desired position is sometimes, so it would be nice to display a marker on the arena representing the goal.
- Improve the node B, distance and average speed are not in real time and are not very accurate
# Flowchart node_A
![Blank diagram.pdf](https://github.com/boez98/RT1_assignment_2/files/10372052/Blank.diagram.pdf
