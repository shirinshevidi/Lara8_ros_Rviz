#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose
import tf.transformations as tf
import numpy as np

def euler_to_quaternion(roll, pitch, yaw):
    quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
    return quaternion

def quaternion_to_z_axis(qx, qy, qz, qw):
    # Calculate the Z-axis vector from the quaternion
    z_x = 2 * (qx * qz + qy * qw)
    z_y = 2 * (qy * qz - qx * qw)
    z_z = 1 - 2 * (qx**2 + qy**2)
    return np.array([z_x, z_y, z_z])

def publish_target_pose():
    # Initialize the moveit_commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('target_pose_publisher', anonymous=True)

    # Instantiate RobotCommander (interface to robot)
    robot = moveit_commander.RobotCommander()

    # Instantiate MoveGroupCommander for controlling a specific group (e.g., manipulator)
    group_name = "arm"  # Change this to your MoveIt group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Instantiate the PlanningSceneInterface for collision objects
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)  # Allow the scene to initialize

    # First target (Euler angles to quaternion conversion)
    roll_1, pitch_1, yaw_1 = 0 * (3.14159265359 / 180), 0 * (3.14159265359 / 180), 0 * (3.14159265359 / 180)
    quaternion_1 = euler_to_quaternion(roll_1, pitch_1, yaw_1)

    target_pose_1 = Pose()
    target_pose_1.orientation.x = quaternion_1[0]
    target_pose_1.orientation.y = quaternion_1[1]
    target_pose_1.orientation.z = quaternion_1[2]
    target_pose_1.orientation.w = quaternion_1[3]
    target_pose_1.position.x = 0.8
    target_pose_1.position.y = -1.0
    target_pose_1.position.z = 0.95

    # Set the first pose target and move the robot
    move_group.set_pose_target(target_pose_1)
    success_1 = move_group.go(wait=True)

    if success_1:
        rospy.loginfo("Successfully moved to the first target pose.")
    else:
        rospy.logwarn("Failed to reach the first target pose.")

    # Second target (Modify for a new target pose)
    roll_2, pitch_2, yaw_2 = -180 * (3.14159265359 / 180), 0 * (3.14159265359 / 180), 90 * (3.14159265359 / 180)
    quaternion_2 = euler_to_quaternion(roll_2, pitch_2, yaw_2)

    target_pose_2 = Pose()
    target_pose_2.orientation.x = quaternion_2[0]
    target_pose_2.orientation.y = quaternion_2[1]
    target_pose_2.orientation.z = quaternion_2[2]
    target_pose_2.orientation.w = quaternion_2[3]
    target_pose_2.position.x = 0.828  # New position
    target_pose_2.position.y = 0.0
    target_pose_2.position.z = 0.623

    # Set the second pose target and move the robot
    move_group.set_pose_target(target_pose_2)
    success_2 = move_group.go(wait=True)

    if success_2:
        rospy.loginfo("Successfully moved to the second target pose.")
        
        # Calculate and print the Z-axis vector of the TCP
        z_axis_vector = quaternion_to_z_axis(target_pose_2.orientation.x,
                                             target_pose_2.orientation.y,
                                             target_pose_2.orientation.z,
                                             target_pose_2.orientation.w)
        rospy.loginfo(f"Z-axis vector of the TCP: {z_axis_vector}")
    else:
        rospy.logwarn("Failed to reach the second target pose.")

    # Stop any leftover motion and clear pose targets
    move_group.stop()
    move_group.clear_pose_targets()

    # Shutdown MoveIt
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        publish_target_pose()
    except rospy.ROSInterruptException:
        pass

