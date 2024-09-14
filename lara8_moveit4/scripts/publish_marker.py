#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_marker():
    rospy.init_node('marker_publisher', anonymous=True)
    pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    marker = Marker()
    marker.header.frame_id = "root_link"  # Replace with your robot's reference frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "target_marker"
    marker.id = 0
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.position = Point(1.0, 0, 0)  # Replace with your target position
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration()

    while not rospy.is_shutdown():
        pub.publish(marker)
        rospy.loginfo("Publishing marker: %s", marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_marker()
    except rospy.ROSInterruptException:
        pass

