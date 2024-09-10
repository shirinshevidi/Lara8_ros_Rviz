#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import PlanningSceneInterface

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
    scene = PlanningSceneInterface()
    rospy.sleep(2)  # Allow the scene to initialize

    # Use the robot's planning frame for the cube
    planning_frame = move_group.get_planning_frame()
    rospy.loginfo(f"Using planning frame: {planning_frame}")

    # Add a cube to the planning scene
    cube_pose = PoseStamped()
    cube_pose.header.frame_id = planning_frame  # Use the robot's planning frame
    cube_pose.pose.position.x = 0.0  # Modify position of the cube
    cube_pose.pose.position.y = 0.0
    cube_pose.pose.position.z = 0.3  # Cube height
    cube_pose.pose.orientation.w = 1.0

    # Add cube as a collision object to the planning scene
    scene.add_box("cube", cube_pose, size=(0.1, 0.1, 0.1))  # Cube size
    rospy.sleep(1)  # Allow time for the cube to be added to the scene

    # Check if the cube is added to the scene
    attached_objects = scene.get_known_object_names()
    if "cube" in attached_objects:
        rospy.loginfo("Cube added to the scene successfully.")
    else:
        rospy.logwarn("Cube was not added to the scene.")

    # Define the target pose for the robot
    target_pose = Pose()
    target_pose.orientation.x = 0.0  # Modify orientation as needed
    target_pose.orientation.y = 0.0
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 1.0
    target_pose.position.x = -0.5  # Modify position to avoid collision with cube
    target_pose.position.y = 0.3
    target_pose.position.z = 0.5

    # Set the pose target for the end-effector or the robot's group
    move_group.set_pose_target(target_pose)

    # Plan the motion to the target pose
    success = move_group.go(wait=True)

    if success:
        rospy.loginfo("Plan successful. No collision detected.")
    else:
        rospy.logwarn("Collision detected or no valid path found!")

    # Ensure no residual motion by stopping any leftover motion
    move_group.stop()

    # Clear targets after planning
    move_group.clear_pose_targets()

    # Shutdown MoveIt
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        publish_target_pose()
    except rospy.ROSInterruptException:
        pass

