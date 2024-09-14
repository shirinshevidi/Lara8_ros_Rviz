#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface
from visualization_msgs.msg import Marker  # Import Marker for color

def add_cube():
    # Initialize moveit_commander and rospy node
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

    # Publisher for visualization markers (to show colored cubes)
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    # Add the first cube (robot base) to the planning scene
    cube_pose = PoseStamped()
    cube_pose.header.frame_id = planning_frame
    cube_pose.pose.position.x = 0.0
    cube_pose.pose.position.y = 0.0
    cube_pose.pose.position.z = -0.25
    cube_pose.pose.orientation.w = 1.0
    scene.add_box("robotbase", cube_pose, size=(0.8, 0.8, 0.5))

    # Create and publish a Marker to display the robot base cube with color in RViz
    cube_marker = Marker()
    cube_marker.header.frame_id = planning_frame
    cube_marker.type = Marker.CUBE
    cube_marker.action = Marker.ADD
    cube_marker.pose = cube_pose.pose
    cube_marker.scale.x = 0.8
    cube_marker.scale.y = 0.8
    cube_marker.scale.z = 0.5

    # Set color for the robot base (green)
    cube_marker.color.r = 0.0
    cube_marker.color.g = 1.0
    cube_marker.color.b = 0.0
    cube_marker.color.a = 1.0  # Fully opaque

    marker_pub.publish(cube_marker)  # Publish the marker to RViz

    # Add the second cube (desk) to the planning scene
    cube_pose = PoseStamped()
    cube_pose.header.frame_id = planning_frame
    cube_pose.pose.position.x = 0.8
    cube_pose.pose.position.y = 0.1
    cube_pose.pose.position.z = -0.14
    cube_pose.pose.orientation.w = 1.0
    scene.add_box("desk", cube_pose, size=(0.8, 1.8, 0.76))

    # Create and publish a Marker to display the desk cube with color in RViz
    cube_marker = Marker()
    cube_marker.header.frame_id = planning_frame
    cube_marker.type = Marker.CUBE
    cube_marker.action = Marker.ADD
    cube_marker.pose = cube_pose.pose
    cube_marker.scale.x = 0.8
    cube_marker.scale.y = 1.8
    cube_marker.scale.z = 0.76

    # Set color for the desk (blue)
    cube_marker.color.r = 0.0
    cube_marker.color.g = 0.0
    cube_marker.color.b = 1.0
    cube_marker.color.a = 1.0  # Fully opaque

    marker_pub.publish(cube_marker)  # Publish the marker to RViz

    rospy.sleep(2)  # Allow more time for the scene to update

    # Verify if cubes were added successfully
    attached_objects = scene.get_known_object_names()
    if "robotbase" in attached_objects:
        rospy.loginfo("robotbase added to the scene successfully.")
    else:
        rospy.logwarn("robotbase was not added to the scene.")

    if "desk" in attached_objects:
        rospy.loginfo("desk added to the scene successfully.")
    else:
        rospy.logwarn("desk was not added to the scene.")


if __name__ == "__main__":
    try:
        add_cube()
    except rospy.ROSInterruptException:
        pass

