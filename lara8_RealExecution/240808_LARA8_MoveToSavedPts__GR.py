#__LARA8 IP: 192.168.2.13
#__LARA8 Port: 30003
#__Robotiq HandE gripper Port='COM3' or 'COM4'
#__To run this file, make sure robot.py is in the same directory as this file

#_(1)Imports
from robot import Robot
import math
import json
import serial
import time
import logging
import binascii
from time import sleep  # for gripper control
 
 
#_(2)Initializing the robot, set to Automatic mode
r = Robot()
r.power_on()
r.set_mode("Automatic")
time.sleep(1)

#_Printing Robot Info
print(r.robot_name)
print(r.dof)
print(r.platform)
print(r.payload)
print(r.kURL)
print(r.robot_urdf_path)
print(r.current_tool)
print(r.connection)
print(r.version)
print(r.robot_status("jointAngles")[0])
print("Starting the robot")


#_(3)Robots' status
tcp_pose = r.get_tcp_pose_quaternion()
flange_pose = r.get_flange_pose()
current_joint_angles = r.get_current_joint_angles()
current_tool_mass = r.get_current_tool_mass()

#_Printing Robots' status
print("TCP Pose: ", tcp_pose)
print("Flange Pose: ", flange_pose)
print("Current Joint Angles: ", current_joint_angles)
print("Current Tool Mass: ", current_tool_mass)


#_(6)moving between saved pts on pendant
r = Robot()
#_Printing TCP
tcp_pose = r.get_tcp_pose()
#_Move to saved points in the pendant
r.move_linear(["P3", "P4"]) 
    
r.stop()

