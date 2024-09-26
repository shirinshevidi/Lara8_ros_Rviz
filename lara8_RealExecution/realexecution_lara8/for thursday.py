#__LARA8 IP: 192.168.2.13
#__LARA8 Port: 30003
#__Robotiq HandE gripper Port='COM3' or 'COM4'
#__To run this file, make sure robot.py is in the same directory as this file

#_(1)Imports
from robot import Robot
import math
import json
#import serial
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
import json

# Load the JSON file (replace 'path_to_json_file.json' with the actual path)
with open(r'C:\myfiles\university\thesis\second_semester\final_review\lara\joint_states.json') as f:

    json_data = json.load(f)


# Initialize the Joint object
# MJ = Joint()
print("Starting the robot")
# Iterate over the dictionaries in the JSON array

for record in json_data:
    # Extract the position data
    position = record.get('position', [])
    print (position) 

    joint_property = {
    "speed": 50.0,
    "acceleration": 50.0,
    "safety_toggle": True,
    "target_joint": position,
    "enable_blending"  : True,
    "current_joint_angles": r.get_current_joint_angles()
    }
    r.move_joint(**joint_property)



r.stop() # if there are multiple motions than,this needs to be called
print("finish")


































