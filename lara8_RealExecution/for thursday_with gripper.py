#__LARA8 IP: 192.168.2.13
#__LARA8 Port: 30003
#__Robotiq HandE gripper Port='COM3' or 'COM4'
#__To run this file, make sure robot.py is in the same directory as this file

#_(1) Imports
from robot import Robot
import math
import json
import serial  # Make sure to import the serial module
import time
import logging
import binascii
from time import sleep  # for gripper control

#_IMPORTANT-2 - Start the gripper before running this script; try changing the port number from COM4 to COM3 or COM5 if it doesn't work. shirin is com7
ser = serial.Serial(port='COM7', baudrate=115200, timeout=1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

def control_gripper(close=True):
    if close:
        print("Close gripper")
        ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")
    else:
        print("Open gripper")
        ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")

    data_raw = ser.readline()
    data = binascii.hexlify(data_raw)
    print(f"Response: {data}")
    time.sleep(2)

#_(2) Initializing the robot, set to Automatic mode
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

# Load the JSON file (replace 'path_to_json_file.json' with the actual path)
with open(r'C:\myfiles\university\thesis\second_semester\final_review\lara\joint_states_withgripper_every_50th.json') as f:
    json_data = json.load(f)

# Initialize variables for gripper status tracking
previous_gripper_status = False  # Initially set to False

# Ensure the gripper is closed initially
control_gripper(close=False)  # Call it before starting the movement loop
#control_gripper(close=True)
print("Starting the robot")
# Iterate over the dictionaries in the JSON array
for record in json_data:
#for record in json_data[::10]: 
    # Extract the position data
    position = record.get('position', [])
    gripper_status = record.get('gripper', False)

    # Check if gripper status has changed
    if gripper_status != previous_gripper_status:
        control_gripper(close=gripper_status)  # Call control_gripper based on current status
        previous_gripper_status = gripper_status  # Update the previous status

    # Move the joint
    joint_property = {
        "speed": 50.0,
        "acceleration": 50.0,
        "safety_toggle": True,
        "target_joint": position,
        "enable_blending": True,
        "current_joint_angles": r.get_current_joint_angles()
    }
    r.move_joint(**joint_property)
    gripper_status = record.get('gripper', False)
    print (position)

r.stop()  # if there are multiple motions then this needs to be called
print("Finish")
