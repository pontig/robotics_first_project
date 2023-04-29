# Odometry of Autonomous Vehicle 

This repository contains the source code of the computation of the odometry of a 4-wheel autonomous vehicle. 

## Code Organization
- `CMakeList.txt` this file contains a description of the project's source files and targets
- `info.txt this` file contrains the registration number, name, last name of the authors 
- `package.xml` this file contains a collection of metadata components 
- `launch/project.launch` this launch file sets the values of node parameters and synchronizes the simulation time with the /clock topic
- `msg/Odom.msg` this file defines the custom odometry message *Odom* 
- `src/odom_node.cpp` this executable contains the code of the initialization of the node, of the declaration and instantiation of the subscriber, service, publishers and transform broadcaster, and of the computation of the odometry and transform
- `srv/ResetOdom.srv` this file defines the service to reset the odometry 

## Installation 

Prerequisites:
  - Linux environment
  - ROS noetic 
  - ROS Publisher on the `/speed_steer` topic or ROS Bag file 
  
Steps:

1. unzip the tar.gz file 
2. start `roscore` 
3. run `roslaunch first_project project.launch` in the command line 

## Method for Odometry Computation
Upon reading a new *Quaternion* message, containing speed (m/s) and steering angle (rad), from the `/speed_steer` topic: 

1. the custom odometry message (x, y, theta, timestamp) is computed assuming an Ackerman Steering model approximated to a bicycle 

1. the custom odometry message is published 

1. the regular odometry message (*Pose*, *Twist*) is computed 

1. the regular odometry message is published 

1. the transform (base frame --> vehicle frame) is comuted 

1. the transform is broadcasted 

