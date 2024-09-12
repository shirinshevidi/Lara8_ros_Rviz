#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import PlanningSceneInterface

def add_cube():
    # Initialize the moveit_commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('target_pose_publisher', anonymous=True)

    # Instantiate RobotCommander (interface to robot)
    robot = moveit_commander.RobotCommander()

    # Instantiate MoveGroupCommander for controlling a specific group (e.g., manipulator)
    group_name = "arm"  # Change this to your MoveIt group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Instantiate the PlanningSceneInterface for collision objects
    scene = PlanningSceneInterface()
    rospy.sleep(2)  # Allow the scene to initialize

    # Use the robot's planning frame for the cube
    planning_frame = move_group.get_planning_frame()
    rospy.loginfo(f"Using planning frame: {planning_frame}")

    # Add a cube to the planning scene
    cube_pose = PoseStamped()
    cube_pose.header.frame_id = planning_frame  # Use the robot's planning frame
    cube_pose.pose.position.x = 0.0
    cube_pose.pose.position.y = 0.0
    cube_pose.pose.position.z = -0.25
    cube_pose.pose.orientation.w = 1.0

    scene.add_box("robotbase", cube_pose, size=(0.8, 0.8, 0.5))
    
    
        # Add a cube to the planning scene
    cube_pose = PoseStamped()
    cube_pose.header.frame_id = planning_frame  # Use the robot's planning frame
    cube_pose.pose.position.x = 0.8
    cube_pose.pose.position.y = 0.1
    cube_pose.pose.position.z = -0.14
    cube_pose.pose.orientation.w = 1.0

    scene.add_box("desk", cube_pose, size=(0.8, 1.8, 0.72))
    
    
    
    
    rospy.sleep(2)  # Allow more time for the scene to update


    attached_objects = scene.get_known_object_names()
    if "robotbase" in attached_objects:
        rospy.loginfo("robotbase added to the scene successfully.")
    else:
        rospy.logwarn("Cube was not added to the scene.")

    attached_objects = scene.get_known_object_names()
    if "desk" in attached_objects:
        rospy.loginfo("desk added to the scene successfully.")
    else:
        rospy.logwarn("Cube was not added to the scene.")

if __name__ == "__main__":
    try:
        add_cube()
    except rospy.ROSInterruptException:
        pass

