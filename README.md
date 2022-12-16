# This is the final code repository for Group 4's RBE 521 (Legged Robotics) 

## Requirements
To run this code ensure that you have installed ROS Noetic (v1.15.4) on Ubuntu 20.04

Additionally run <pip install -r requirements.txt> to ensure you have the proper Python libraries

## Embedded Code
(If you have the physial robot)

Use the PlatformIO VSCode extension to upload the Embedded Code folder to the ESP32

## Execution
To launch all the reqired nodes follow these commands:

1. `cd ./ROS Nodes`
2. `catkin_make`
3. `source devel/setup.sh`
4. `roscore`
5. `rosrun legged_robot legged_robot.py`

At this point you should have the legged_robot control node running, from here you should see a visualization of the robot and can control it by publishing messages to the /robot_pose or /robot_walk topics (See Control section on the specifics of these topics). 

To connect with the physical robot execute the following command

6. `rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0 _baud:=115200`

You may need to change your port to check the port it is connected to run 
`ls /dev/` without the ESP32 connected and `ls /dev/` again after connecting it. You should see a new item popup after connecting and that will be your port.

## Control
If you want to control the robot manually you can send messages to the /robot_pose or /robot_walk topics. 

/robot_pose:
This topic uses the geometry_msg/Pose message. Where the position submessage controls the Position of the center of the body w.r.t the global origin. the Orientation submessage controls the Euler Angles (XYZ) of the body.

/robot_walk:
This topic uses the std_msg/Vector3 messgae. where the x and y submessages correspond to the speed of the robot in each respective direction. The z submessage is interpreted as the duration of the motion in seconds.

