#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import PlanningSceneInterface
import tf.transformations as tf

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

    # Define the base positions for elements
    positions = [
        (0.7, 0.1, 0.4),
        (0.96, 0.1, 0.4),
        (0.83, 0.1, 0.58),
        (0.93, 0.1, 0.605),
        (0.828, 0.1, 0.623)
    ]

    # Define the size and rotation (Euler angles) for the elements
    rotations = [
        (0, 0, 0),
        (0, 0, 0),
        (90, 0, 90),
        (90, 90, 0),
        (86, -180, -90)
    ]

    sizes = [
        (0.04, 0.03, 0.33),
        (0.04, 0.03, 0.33),
        (0.07, 0.03, 0.33),
        (0.02, 0.03, 0.285),
        (0.04, 0.03, 0.33)
    ]

    # Modify the y-coordinates for the second and third groups
    y_modifications = [-0.2, 0.1, 0.4]

    element_index = 1  # To name elements sequentially

    # Loop to add the 15 elements with adjusted y-coordinates
    for y_mod in y_modifications:
        for i, (pos_x, _, pos_z) in enumerate(positions):
            roll, pitch, yaw = rotations[i]
            size = sizes[i]

            # Create PoseStamped for each element
            cube_pose = PoseStamped()
            cube_pose.header.frame_id = planning_frame
            cube_pose.pose.position.x = pos_x
            cube_pose.pose.position.y = y_mod  # Apply modified y-coordinates
            cube_pose.pose.position.z = pos_z

            # Convert Euler angles (roll, pitch, yaw) to a quaternion
            quaternion = tf.quaternion_from_euler(roll * (3.14159 / 180), pitch * (3.14159 / 180), yaw * (3.14159 / 180))
            cube_pose.pose.orientation.x = quaternion[0]
            cube_pose.pose.orientation.y = quaternion[1]
            cube_pose.pose.orientation.z = quaternion[2]
            cube_pose.pose.orientation.w = quaternion[3]

            # Add each cube with a sequential name (element1 to element15)
            scene.add_box(f"element{element_index}", cube_pose, size=size)
            element_index += 1  # Increment element index
    
    rospy.sleep(2)  # Allow more time for the scene to update

    # Check if objects were successfully added
    attached_objects = scene.get_known_object_names()
    if "robotbase" in attached_objects:
        rospy.loginfo("robotbase added to the scene successfully.")
    else:
        rospy.logwarn("Cube was not added to the scene.")

    if "desk" in attached_objects:
        rospy.loginfo("desk added to the scene successfully.")
    else:
        rospy.logwarn("Cube was not added to the scene.")

if __name__ == "__main__":
    try:
        add_cube()
    except rospy.ROSInterruptException:
        pass
