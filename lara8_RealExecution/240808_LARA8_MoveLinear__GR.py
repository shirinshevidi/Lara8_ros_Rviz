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


#_(3) Printing Robot Info
print(r.robot_name)
print(r.dof)
print(r.platform)
print(r.payload)
print(r.kURL)
print(r.robot_urdf_path)
print(r.current_tool)
print(r.connection)
print(r.version)

print("Starting the robot")
print(r.robot_status("jointAngles")[0])


#_Printing Robot Status and Mode
#_Printing TCP
tcp_pose = r.get_tcp_pose()
print(f"Current TCP - {tcp_pose}")

#_Cartesian Points for Linear Movement
#_IMPORTANT - Values are in Meters and Angles are in Radians
#_have always at least 3 targets.
r = Robot()
linear_property = {
    "speed": 0.25,    
    "acceleration": 0.1,
    "rotation_speed": 0.5,
    "blending": True,
    "blending_mode": 2,
    "blend_radius": 0.005,
    "target_pose": [
        [-0.028640, -0.718971, 0.686, -3.141593, 0.0, -0.8726646],
        [ -0.028640, -0.718971, 0.446, -3.141593, 0.0, -0.8726646],
        [ -0.028640, -0.718971, 0.546, -3.141593, 0.0, -0.8726646]
    ],
    "current_joint_angles": r.get_current_joint_angles()
}

r.move_linear(**linear_property)

r.stop()


