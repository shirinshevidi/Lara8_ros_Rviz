#__LARA8 IP: 192.168.2.13
#__LARA8 Port: 30003
#__Robotiq HandE gripper Port='COM3' or 'COM4'
#__To run this file, make sure robot.py is in the same directory as this file

# Imports
from robot import Robot
import math
import json
import serial
import time
import binascii
from time import sleep  # for gripper control

#_Initial SetUp
r = Robot()

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


#_Printing Robot Status and Mode
print(r.robot_status("jointAngles")[0])
r.set_mode("Automatic")
print("Starting the robot")

#_Printing TCP
tcp_pose = r.get_tcp_pose()
print(f"Current TCP - {tcp_pose}")

#_IMPORTANT - Values are in Meters and Angles are in Radians
linear_property = {
    "speed": 0.25,
    "acceleration": 0.1,
    "blend_radius": 0.005,
    "target_pose": [
        [0.183833, -0.676289, 0.527345, -3.1281885, 0.00, -1.094199], #Pt.1 on LARA8 pendant
        [-0.122246, -0.746377, 0.599107, -3.1372991, 0.006021386, -2.6173307] #Safety Pt. on LARA8 pendant
    ],
    "current_joint_angles": r.robot_status("jointAngles")
}

#_IMPORTANT-2 - Start the gripper before running this script; try changing the port number from COM4 to COM3 or COM5 if it doesn't work
ser = serial.Serial(port='COM4', baudrate=115200, timeout=1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

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

# Move to first point
linear_property["target_pose"] = [linear_property["target_pose"][0]]
r.move_linear(**linear_property)
control_gripper(close=True)
control_gripper(close=False)

# Move to second point
linear_property["target_pose"] = [linear_property["target_pose"][1]]
r.move_linear(**linear_property)
control_gripper(close=True)
control_gripper(close=False)

print("Operation completed")
