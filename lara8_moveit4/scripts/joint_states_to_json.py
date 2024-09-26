#!/usr/bin/env python3

import rospy
import json
from sensor_msgs.msg import JointState
import os

# Initialize a list to store multiple joint state messages
joint_states_list = []

# Path to the external gripper status JSON file
gripper_status_file = "/home/shirin/Unity-Robotics-Hub/tutorials/pick_and_place/ROS/src/lara8_moveit4/scripts/gripper_status.json"

def get_gripper_status():
    """Reads the gripper status from an external JSON file."""
    if os.path.exists(gripper_status_file):
        with open(gripper_status_file, 'r') as file:
            data = json.load(file)
            status= data.get('gripper_status') 
            if status not in [True, False]:
             status = False

   
            return status  # Default to False if the key is not found


def callback(data):
    # Convert JointState message to a dictionary
    joint_data = {
        "header": {
            "seq": data.header.seq,
            "stamp": {
                "secs": data.header.stamp.secs,
                "nsecs": data.header.stamp.nsecs
            },
            "frame_id": data.header.frame_id
        },
        "name": data.name,
        "position": list(data.position),
        "velocity": list(data.velocity),
        "effort": list(data.effort),
        "gripper": get_gripper_status()  # Add gripper status
    }

    # Append the new data to the list
    if isinstance(joint_states_list, list):
        joint_states_list.append(joint_data)

    # Save the list of joint states to a JSON file
    with open('joint_states.json', 'w') as json_file:
        json.dump(joint_states_list, json_file, indent=4)

def listener():
    rospy.init_node('joint_states_to_json', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    # If the JSON file already exists, load the existing data
    if os.path.exists('joint_states.json'):
        with open('joint_states.json', 'r') as json_file:
            try:
                loaded_data = json.load(json_file)
                # Ensure the loaded data is a list
                if isinstance(loaded_data, list):
                    joint_states_list.extend(loaded_data)
                else:
                    rospy.logwarn("Invalid JSON structure, starting with an empty list.")
                    joint_states_list = []
            except json.JSONDecodeError:
                rospy.logwarn("Invalid JSON file, starting fresh.")
                joint_states_list = []

    listener()
