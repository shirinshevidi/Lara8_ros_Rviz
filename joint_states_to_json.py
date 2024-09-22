#!/usr/bin/env python3

import rospy
import json
from sensor_msgs.msg import JointState
import os

# Initialize a list to store multiple joint state messages
joint_states_list = []

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
        "effort": list(data.effort)
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
