#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose
import tf.transformations as tf

def euler_to_quaternion(roll, pitch, yaw):
    quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
    return quaternion

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
    roll_1, pitch_1, yaw_1 = -90 * (3.14159265359 / 180), 0 * (3.14159265359 / 180), 0 * (3.14159265359 / 180)
    quaternion_1 = euler_to_quaternion(roll_1, pitch_1, yaw_1)

    target_pose_1 = Pose()
    target_pose_1.orientation.x = quaternion_1[0]
    target_pose_1.orientation.y = quaternion_1[1]
    target_pose_1.orientation.z = quaternion_1[2]
    target_pose_1.orientation.w = quaternion_1[3]
    target_pose_1.position.x = 0.8
    target_pose_1.position.y = 0.0
    target_pose_1.position.z = 0.45

    # Set the first pose target and move the robot
    move_group.set_pose_target(target_pose_1)
    success_1 = move_group.go(wait=True)

    if success_1:
        rospy.loginfo("Successfully moved to the first target pose.")
        
        # Ensure Box_1 is in the scene
        rospy.sleep(1)  # Give some time to make sure the scene is updated
        if "Box_1" in scene.get_known_object_names():
            link_name = "tcp"  # Replace with your robot's actual end-effector link
              # Fetch all the link names of the robot
            touch_links = robot.get_link_names()  # Get all robot link names
            scene.attach_box(link_name, "Box_1", touch_links=touch_links)

            rospy.loginfo("Box_1 attached to the link.")
        else:
            rospy.logwarn("Box_1 is not in the scene.")
    else:
        rospy.logwarn("Failed to reach the first target pose.")

    # Third target (Euler angles to quaternion conversion)
    roll_3, pitch_3, yaw_3 = -90 * (3.14159265359 / 180), 0 * (3.14159265359 / 180), 0 * (3.14159265359 / 180)
    quaternion_3 = euler_to_quaternion(roll_3, pitch_3, yaw_3)

    target_pose_3 = Pose()
    target_pose_3.orientation.x = quaternion_3[0]
    target_pose_3.orientation.y = quaternion_3[1]
    target_pose_3.orientation.z = quaternion_3[2]
    target_pose_3.orientation.w = quaternion_3[3]
    target_pose_3.position.x = 0.8
    target_pose_3.position.y = 0.0
    target_pose_3.position.z = 0.6

    # Set the third pose target and move the robot
    move_group.set_pose_target(target_pose_3)
    success_3 = move_group.go(wait=True)

    if success_3:
        rospy.loginfo("Successfully moved to the third target pose.")
    else:
        rospy.logwarn("Failed to reach the third target pose.")

    # Second target (Modify for a new target pose)
    roll_2, pitch_2, yaw_2 = 0 * (3.14159265359 / 180), 90 * (3.14159265359 / 180), 0 * (3.14159265359 / 180)
    quaternion_2 = euler_to_quaternion(roll_2, pitch_2, yaw_2)

    target_pose_2 = Pose()
    target_pose_2.orientation.x = quaternion_2[0]
    target_pose_2.orientation.y = quaternion_2[1]
    target_pose_2.orientation.z = quaternion_2[2]
    target_pose_2.orientation.w = quaternion_2[3]
    target_pose_2.position.x = 0.6  # New position
    target_pose_2.position.y = -0.2
    target_pose_2.position.z = 0.8

    # Set the second pose target and move the robot
    move_group.set_pose_target(target_pose_2)
    success_2 = move_group.go(wait=True)

    if success_2:
        rospy.loginfo("Successfully moved to the second target pose.")
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

