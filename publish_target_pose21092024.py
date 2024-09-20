#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose
import tf.transformations as tf
import numpy as np
import json  


# json from planning Agent 
json_file = "/home/shirin/Unity-Robotics-Hub/tutorials/pick_and_place/ROS/src/lara8_moveit4/scripts/planning.json"  # Update with the path to your JSON file

# Initialize moveit_commander and rospy once
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('robot_controller', anonymous=True)  # Use a single node name

# Instantiate RobotCommander and PlanningSceneInterface globally
scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()
link_name = "tcp"
group_name = "arm"  # Change this to your MoveIt group name
move_group = moveit_commander.MoveGroupCommander(group_name)

def move_in_cartesian_path(group, move_distance_x, move_distance_y, move_distance_z):
    waypoints = []

    # Get the current pose of the end effector
    current_pose = group.get_current_pose().pose

    # Define the new pose (move in x, y, z directions)
    target_pose = Pose()
    target_pose.orientation = current_pose.orientation  # Keep orientation the same
    target_pose.position.x = current_pose.position.x + move_distance_x
    target_pose.position.y = current_pose.position.y + move_distance_y
    target_pose.position.z = current_pose.position.z + move_distance_z

    # Append the target pose as a waypoint
    waypoints.append(target_pose)

    # Compute the Cartesian path
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,   # Waypoints to follow
        0.01,        # Resolution of the path (in meters)
        0.0          # Jump threshold (disable avoidance of kinematic jumps)
    )

    if fraction == 1.0:
        #rospy.loginfo("Cartesian path planned successfully!")
        group.execute(plan, wait=True)
        return True
    else:
        #rospy.logwarn("Failed to plan the entire Cartesian path.")
        return False
    
def euler_to_quaternion(roll, pitch, yaw):
    quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
    return quaternion

def quaternion_to_z_axis(qx, qy, qz, qw):
    # Calculate the Z-axis vector from the quaternion
    z_x = 2 * (qx * qz + qy * qw)
    z_y = 2 * (qy * qz - qx * qw)
    z_z = 1 - 2 * (qx**2 + qy**2)
    return np.array([z_x, z_y, z_z])

def moveto(pose_element, element_name):
    A, B, C, X, Y, Z = pose_element

    #group_name = "arm"  # Change this to your MoveIt group name
    #move_group = moveit_commander.MoveGroupCommander(group_name)

    # Convert Euler angles to quaternion
    roll, pitch, yaw = A * (3.14159265359 / 180), B * (3.14159265359 / 180), C * (3.14159265359 / 180)
    quaternion = euler_to_quaternion(roll, pitch, yaw)

    # Set the target pose
    target_pose = Pose()
    target_pose.orientation.x = quaternion[0]
    target_pose.orientation.y = quaternion[1]
    target_pose.orientation.z = quaternion[2]
    target_pose.orientation.w = quaternion[3]
    target_pose.position.x = X
    target_pose.position.y = Y
    target_pose.position.z = Z

    # Set the pose target and move the robot
    move_group.set_pose_target(target_pose)
    success = move_group.go(wait=True)

    if success:
        rospy.loginfo(f"Successfully moved to {element_name} postion.")
    else:
        rospy.logwarn(f"Failed to reach the {element_name} postion.")

    # Stop any leftover motion and clear pose targets
    move_group.stop()
    move_group.clear_pose_targets()

    return success

def picking(pose_element, element_name):
    picking_offset = 0.08
    moveup = 0.14
    A, B, C, X, Y, Z = pose_element

    # Convert Euler angles to quaternion
    roll, pitch, yaw = A * (3.14159265359 / 180), B * (3.14159265359 / 180), C * (3.14159265359 / 180)
    quaternion = euler_to_quaternion(roll, pitch, yaw)
    axes = quaternion_to_z_axis(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
    X_offset = X - ( picking_offset * axes[0] )
    Y_offset = Y - ( picking_offset * axes[1] )
    Z_offset = Z - ( picking_offset * axes[2] )
    
    # if  the element is already attached to the TCP , we skip moveto(pose_element_offset) and attach step, directly start with move_in_cartesian_path(move_group, move_distance_x=0.0, move_distance_y=0.0, move_distance_z= moveup)


    pose_element_offset = (A, B, C, X_offset, Y_offset, Z_offset)
    rospy.loginfo(pose_element_offset)
    moveto(pose_element_offset, "pose_element_offset")

    # if  moveto(pose_element_offset) not successful it should check the other sides 


    # Move the robot in Cartesian space
    success=  move_in_cartesian_path(move_group, move_distance_x= picking_offset * axes[0] , move_distance_y= picking_offset * axes[1], move_distance_z= picking_offset * axes[2] )

    #moveto(pose_element)

    if  success:
        rospy.loginfo("Robot is in grab plane successfully.")
        rospy.sleep(1)

        #link_name = "tcp"
        touch_links = robot.get_link_names()
        scene.attach_box(link_name, element_name, touch_links=touch_links)

        rospy.loginfo("The element is grabbed.")

        #Z_moveup = Z + moveup
        #pose_element_moveup = (A, B, C, X, Y, Z_moveup)
        move_in_cartesian_path(move_group, move_distance_x=0.0, move_distance_y=0.0, move_distance_z= moveup)

    else:
        rospy.logwarn("Failed to reach the grab plane.")

def holding(pose_element, element_name):
    holding_offset = 0.08
    
    A, B, C, X, Y, Z = pose_element

    # Convert Euler angles to quaternion
    roll, pitch, yaw = A * (3.14159265359 / 180), B * (3.14159265359 / 180), C * (3.14159265359 / 180)
    quaternion = euler_to_quaternion(roll, pitch, yaw)
    axes = quaternion_to_z_axis(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
    X_offset = X - ( holding_offset * axes[0] )
    Y_offset = Y - ( holding_offset * axes[1] )
    Z_offset = Z - ( holding_offset * axes[2] )

    pose_element_offset = (A, B, C, X_offset, Y_offset, Z_offset)
    rospy.loginfo(pose_element_offset)
    
    moveto(pose_element_offset, "pose_element_offset")
    # if  moveto(pose_element_offset) not successful it should check the other sides 


    #group_name = "arm"  # Change this to your MoveIt group name
    #move_group = moveit_commander.MoveGroupCommander(group_name)

    # Move the robot in Cartesian space
    success=  move_in_cartesian_path(move_group, move_distance_x= holding_offset * axes[0] , move_distance_y= holding_offset * axes[1], move_distance_z= holding_offset * axes[2] )

    if  success:
        rospy.loginfo("Robot is in support plane successfully.")
        rospy.sleep(1)

        link_name = "tcp"
        touch_links = robot.get_link_names()
        scene.attach_box(link_name, element_name, touch_links=touch_links)

        rospy.loginfo(f"{element_name} is supported!")
                #waiting for human

        with open(json_file, 'r') as file:
            data = json.load(file)
        while data.get('attach_element3') == False:       #false , means robot shoud and human is  still working
            rospy.loginfo("Waiting for human... ")
            rospy.sleep(1)  # Wait for 1 second before checking again
            
            # Optionally, you can re-read the JSON file to check for updated value
            with open(json_file, 'r') as file:
                data = json.load(file)

        # Once 'attach_element3' becomes True, remove the attached object
        rospy.loginfo(f"Robot is releasing {element_name} ")
        scene.remove_attached_object(link_name, element_name)
        rospy.loginfo(f" {element_name} is released from robot ")

        move_in_cartesian_path(move_group, move_distance_x= - holding_offset * axes[0] , move_distance_y= - holding_offset * axes[1], move_distance_z= - holding_offset * axes[2] )

    else:
        rospy.logwarn("Failed to reach the support plane.")

def placing(pose_element, element_name):
   
    placing_offset = 0.08
    
    moveto(safepoint, "safepoint")
    moveto(storage_offset, "storage_offset")

    #storage, release point 
    move_in_cartesian_path(move_group, move_distance_x=0.0, move_distance_y=0.0, move_distance_z= - placing_offset )
    
    scene.remove_attached_object(link_name, element_name)
    rospy.loginfo(f" {element_name} is released from robot ")

    move_in_cartesian_path(move_group, move_distance_x=0.0, move_distance_y=0.0, move_distance_z= placing_offset )
    moveto(safepoint, "safepoint")
    rospy.loginfo(f" {element_name} is disassembled successfully! ")

 
if __name__ == "__main__":
    try:
        home = (-180, 0, 90, 0.25, 0.1, 0.80)
        safepoint = (-180, 0, 90, 0.45, 0.4, 0.60)
        storage_offset = (-180, 0, 90, 0.8, 0.5, 0.28)
        element3 = (90, 90, 0, 0.83, 0.1, 0.58)

        moveto(home,"home" )
        holding(element3, "element3")
        #moveto(home, "home")
        #placing(element3, "element3")
        moveto(home, "home")

    except rospy.ROSInterruptException:
        pass
