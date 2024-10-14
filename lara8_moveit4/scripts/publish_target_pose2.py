#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose

# Initialize moveit_commander and rospy once
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('cartesian_path_planner', anonymous=True)

# Instantiate RobotCommander and PlanningSceneInterface globally
scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()

def move_in_cartesian_path(group, move_distance_x, move_distance_y, move_distance_z):
    waypoints = []

    # Get the current pose of the end effector
    current_pose = group.get_current_pose().pose

    # Define the new pose (move 5 units in x direction for example)
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
        rospy.loginfo("Cartesian path planned successfully!")
        group.execute(plan, wait=True)
    else:
        rospy.logwarn("Failed to plan the entire Cartesian path.")

if __name__ == "__main__":
    try:
        # Define the MoveGroupCommander for the arm
        group_name = "arm"  # Change this to your MoveIt group name
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # Move the robot in Cartesian space by 5 units in the x-direction
        move_in_cartesian_path(move_group, move_distance_x=0.5, move_distance_y=0.0, move_distance_z=0.0)

    except rospy.ROSInterruptException:
        pass

    finally:
        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()

