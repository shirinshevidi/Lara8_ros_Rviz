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
json_file_gripper = "/home/shirin/Unity-Robotics-Hub/tutorials/pick_and_place/ROS/src/lara8_moveit4/scripts/gripper_status.json"  # Update with the path to your JSON file
# Initialize moveit_commander and rospy once
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('robot_controller', anonymous=True)  # Use a single node name

# Instantiate RobotCommander and PlanningSceneInterface globally
scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()
link_name = "tcp"
group_name = "arm"  # Change this to your MoveIt group name
move_group = moveit_commander.MoveGroupCommander(group_name)


def update_gripper_status(status, file_path='/home/shirin/Unity-Robotics-Hub/tutorials/pick_and_place/ROS/src/lara8_moveit4/scripts/gripper_status.json'):
    # Create a dictionary to store the status
    data = {'gripper_status': status}
    
    # Write the dictionary to a JSON file
    with open(file_path, 'w') as json_file_gripper :
        json.dump(data, json_file_gripper , indent=4)
    
    print(f"Gripper status updated to: {status}")

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

def quaternion_to_y_axis(qx, qy, qz, qw):
    y_x = 2 * (qx * qy - qz * qw)
    y_y = 1 - 2 * (qx**2 + qz**2)
    y_z = 2 * (qy * qz + qx * qw)
    return np.array([y_x, y_y, y_z])

def moveto(pose_element, element_name):

    holding_offset = 0.08
    global holding_status
    global current_axes 
    global supported_element
    if holding_status ==1 :
        rospy.loginfo(f"Robot is releasing {supported_element} ")
        link_name = "tcp"
        touch_links = robot.get_link_names()
        scene.remove_attached_object(link_name, supported_element)
        gripper_status = False  # or False, depending on your code logic
        update_gripper_status(gripper_status)
        rospy.loginfo(f" {supported_element} is released from robot ")
        move_in_cartesian_path(move_group, move_distance_x= - holding_offset * current_axes [0] , move_distance_y= - holding_offset * current_axes [1], move_distance_z= - holding_offset * current_axes[2] )        
        holding_status =0 

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
    
    moveup = 0.14
    picking_offset = 0.08
    global iteration
    global holding_status

    if holding_status== 1:
        move_in_cartesian_path(move_group, move_distance_x=0.0, move_distance_y=0.0, move_distance_z= moveup)
        rospy.loginfo("Robot is picking successfully.")
        holding_status= 0
    else :
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
        success = moveto(pose_element_offset, "pose_element_offset")

        # if  moveto(pose_element_offset) not successful it should check the other sides 
        if success : 
            
            rospy.loginfo(f"the robot reaches the offset traget plane with {iteration * 90} degree rotation ")

            # Move the robot in Cartesian space
            success=  move_in_cartesian_path(move_group, move_distance_x= picking_offset * axes[0] , move_distance_y= picking_offset * axes[1], move_distance_z= picking_offset * axes[2] )

            if  success:
                link_name = "tcp"
                touch_links = robot.get_link_names()
                scene.attach_box(link_name, element_name, touch_links=touch_links)
                gripper_status = True  # or False, depending on your code logic
                update_gripper_status(gripper_status)

                rospy.loginfo(f"{element_name} is grabbed!")
                move_in_cartesian_path(move_group, move_distance_x=0.0, move_distance_y=0.0, move_distance_z= moveup)               
                if iteration > 0: 
                    rospy.loginfo(f"the first defined target was not reachable but finally Robot picks successfully with {iteration * 90} degree rotation ")
                else:
                    rospy.loginfo("Robot is picking successfully.")
                rospy.sleep(1)


    
            else: 
                rospy.logwarn(f"Failed to grab element with {iteration * 90} degree rotation from target plane")
                iteration += 1

                if iteration == 4 :
                    rospy.logwarn("choose another element to select. the element is not reachable.")
                else :
                    # Get the Y-axis vector for the given quaternion
                    y_axis = quaternion_to_y_axis(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

                    # Define the angle by which you want to rotate around the Y-axis (in degrees)
                    rotation_angle_degrees = -90
                    rotation_angle_radians = np.deg2rad(rotation_angle_degrees)

                    # Create the quaternion for a rotation around the Y-axis
                    rotation_quaternion = tf.quaternion_about_axis(rotation_angle_radians, y_axis)  # Rotate around the global Y-axis

                    # Combine the original quaternion with the new rotation
                    new_quaternion = tf.quaternion_multiply(rotation_quaternion, quaternion)

                    # Convert the new quaternion to Euler angles (in radians)
                    new_euler = tf.euler_from_quaternion(new_quaternion)

                    # Convert the new Euler angles back to degrees
                    new_euler_degrees = np.rad2deg(new_euler)
                    new_side_element = (new_euler_degrees[0] ,  new_euler_degrees[1] , new_euler_degrees[2] , X, Y, Z )
                    picking(new_side_element  , element_name )
        else:

            rospy.logwarn(f"the traget with {iteration * 90} degree rotation is not reachable")
            iteration += 1

            if iteration == 4 :
                rospy.logwarn("choose another element to select. the element is not reachable.")
            else :
                # Get the Y-axis vector for the given quaternion
                y_axis = quaternion_to_y_axis(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

                # Define the angle by which you want to rotate around the Y-axis (in degrees)
                rotation_angle_degrees = -90
                rotation_angle_radians = np.deg2rad(rotation_angle_degrees)

                # Create the quaternion for a rotation around the Y-axis
                rotation_quaternion = tf.quaternion_about_axis(rotation_angle_radians, y_axis)  # Rotate around the global Y-axis

                # Combine the original quaternion with the new rotation
                new_quaternion = tf.quaternion_multiply(rotation_quaternion, quaternion)

                # Convert the new quaternion to Euler angles (in radians)
                new_euler = tf.euler_from_quaternion(new_quaternion)

                # Convert the new Euler angles back to degrees
                new_euler_degrees = np.rad2deg(new_euler)
                new_side_element = (new_euler_degrees[0] ,  new_euler_degrees[1] , new_euler_degrees[2] , X, Y, Z )
                picking(new_side_element  , element_name )

def holding(pose_element, element_name):
    global iteration
    global current_axes
    global holding_status
    global supported_element

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
    #rospy.loginfo(pose_element_offset)
    
    success = moveto(pose_element_offset, "pose_element_offset")
    # if  moveto(pose_element_offset) not successful it should check the other sides 

    if success : 
        rospy.loginfo(f"the robot reaches the offset traget plane with {iteration * 90} degree rotation ")
        # Move the robot in Cartesian space
        success=  move_in_cartesian_path(move_group, move_distance_x= holding_offset * axes[0] , move_distance_y= holding_offset * axes[1], move_distance_z= holding_offset * axes[2] )

        if  success:
            if iteration > 0: 
                  rospy.loginfo(f"the first defined target was not reachable but finally  Robot holds successfully with {iteration * 90} degree rotation ")
            else:
                rospy.loginfo("Robot is holding successfully.")

            rospy.sleep(1)

            link_name = "tcp"
            touch_links = robot.get_link_names()
            scene.attach_box(link_name, element_name, touch_links=touch_links)
            
            gripper_status = True  
            update_gripper_status(gripper_status)
            

            rospy.loginfo(f"{element_name} is supported!")
                    #waiting for human

            with open(json_file, 'r') as file:
                data = json.load(file)
            while data.get('human_working') == True:       #true , means robot should wait and human is  still working
                rospy.loginfo("Waiting for human... ")
                rospy.sleep(1)  # Wait for 1 second before checking again
                # Optionally, you can re-read the JSON file to check for updated value
                with open(json_file, 'r') as file:
                    data = json.load(file)

            
            holding_status= 1
            current_axes= axes
            supported_element = element_name


        else:
            rospy.logwarn(f"Failed to the support element with {iteration * 90} degree rotation from target plane")
            # moveto(home, "home")
            iteration += 1

            if iteration == 4 :
                rospy.logwarn("choose another element to select. the element is not reachable.")
            else :
                # Get the Y-axis vector for the given quaternion
                y_axis = quaternion_to_y_axis(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

                # Define the angle by which you want to rotate around the Y-axis (in degrees)
                rotation_angle_degrees = -90
                rotation_angle_radians = np.deg2rad(rotation_angle_degrees)

                # Create the quaternion for a rotation around the Y-axis
                rotation_quaternion = tf.quaternion_about_axis(rotation_angle_radians, y_axis)  # Rotate around the global Y-axis

                # Combine the original quaternion with the new rotation
                new_quaternion = tf.quaternion_multiply(rotation_quaternion, quaternion)

                # Convert the new quaternion to Euler angles (in radians)
                new_euler = tf.euler_from_quaternion(new_quaternion)

                # Convert the new Euler angles back to degrees
                new_euler_degrees = np.rad2deg(new_euler)
                new_side_element = (new_euler_degrees[0] ,  new_euler_degrees[1] , new_euler_degrees[2] , X, Y, Z )
                holding(new_side_element , element_name )
    else:

        rospy.logwarn(f"the traget with {iteration * 90} degree rotation is not reachable")
        iteration += 1
       
        if iteration == 4 :
            rospy.logwarn("choose another element to select. the element is not reachable.")
        else :
            # Get the Y-axis vector for the given quaternion
            y_axis = quaternion_to_y_axis(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

            # Define the angle by which you want to rotate around the Y-axis (in degrees)
            rotation_angle_degrees = -90
            rotation_angle_radians = np.deg2rad(rotation_angle_degrees)

            # Create the quaternion for a rotation around the Y-axis
            rotation_quaternion = tf.quaternion_about_axis(rotation_angle_radians, y_axis)  # Rotate around the global Y-axis

            # Combine the original quaternion with the new rotation
            new_quaternion = tf.quaternion_multiply(rotation_quaternion, quaternion)

            # Convert the new quaternion to Euler angles (in radians)
            new_euler = tf.euler_from_quaternion(new_quaternion)

            # Convert the new Euler angles back to degrees
            new_euler_degrees = np.rad2deg(new_euler)
            new_side_element = (new_euler_degrees[0] ,  new_euler_degrees[1] , new_euler_degrees[2] , X, Y, Z )
            holding(new_side_element , element_name  )
            
             
def placing(pose_element, element_name):
   
    placing_offset = 0.08
    
    moveto(safepoint, "safepoint")
    success = moveto(storage_offset, "storage_offset")
    if success :
        #storage, release point 
        move_in_cartesian_path(move_group, move_distance_x=0.0, move_distance_y=0.0, move_distance_z= - placing_offset )

        scene.remove_attached_object(link_name, element_name)
        gripper_status = False # or False, depending on your code logic
        update_gripper_status(gripper_status)
        rospy.loginfo(f" {element_name} is released from robot ")

        move_in_cartesian_path(move_group, move_distance_x=0.0, move_distance_y=0.0, move_distance_z= placing_offset )
        moveto(safepoint, "safepoint")
        rospy.loginfo(f" {element_name} is disassembled successfully! ")

 
if __name__ == "__main__":
    try:
        # Initialization variables
        holding_status = 0
        iteration = 0 
        home = (-180, 0, 90, 0.25, 0.1, 0.80)
        safepoint = (-180, 0, 90, 0.45, 0.4, 0.60)
        storage_offset = (-180, 0, 90, 0.8, 0.5, 0.28)
        
        # Define your elements
        element1 = (90, 0, 180, 0.7, 0.1, 0.4)
        element2 = (90, 0, 0, 0.96, 0.1, 0.4)
        element3 = (90, -90, 0, 0.83, 0.1, 0.58)

        # Dictionary mapping element names to their coordinates
        elements_dict = {
            "element1": element1,
            "element2": element2,
            "element3": element3
        }

        # Read the command sequence from the JSON file

        with open(json_file, 'r') as file:
            data = json.load(file)
            selected_element_name = data.get('selected_element')  # e.g., 'element1', 'element2', etc.
            planning_sequence = data.get('planning_sequence', [])

        if selected_element_name in elements_dict:
            selected_element = elements_dict[selected_element_name]
        else:
            print(f"Error: {selected_element_name} is not a valid element.")

        # Define a mapping of pose names to their corresponding tuples
        poses_dict = {
            "home": home,
            "safepoint": safepoint,
            "storage_offset": storage_offset,
            **elements_dict  # Merge elements_dict into poses_dict
        }

        # Iterate through the command sequence and dynamically call functions with the right arguments
        for command in planning_sequence:
            # print(f"dddddd: {selected_element_name} is not a valid element.")
            if command.startswith("moveto("):
                # Extract the pose name from the command
                pose_name = command[7:-1]  # Get the content inside moveto()
                if pose_name in poses_dict:
                    moveto(poses_dict[pose_name], pose_name)
                else:
                    print(f"Error: {pose_name} is not a valid pose.")
            elif command == "holding":
                holding(selected_element, selected_element_name)
            elif command == "picking":
                picking(selected_element, selected_element_name)
            elif command == "placing":
                placing(selected_element, selected_element_name)

        # Final movement back to home
        moveto(home, "home")

    except rospy.ROSInterruptException:
        pass
