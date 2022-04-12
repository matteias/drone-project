#!/usr/bin/env python
import rospy
from aruco_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseWithCovariance, Point, Quaternion, TransformStamped
import tf2_ros



def send_marker_transform(msg):
    for marker in msg.markers:
        t = TransformStamped()
        t.header.stamp = marker.header.stamp
        if marker.id != 0:
            t.header.frame_id = 'cf1/camera_link'
            t.child_frame_id = 'sign/detected'+str(marker.id)
            t.transform.translation.x = marker.pose.pose.position.x
            t.transform.translation.y = marker.pose.pose.position.y
            t.transform.translation.z = marker.pose.pose.position.z
            t.transform.rotation.x = marker.pose.pose.orientation.x
            t.transform.rotation.y = marker.pose.pose.orientation.y
            t.transform.rotation.z = marker.pose.pose.orientation.z
            t.transform.rotation.w = marker.pose.pose.orientation.w
            broadcaster.sendTransform(t)


rospy.init_node('sign_publisher')
pose_pubby = rospy.Subscriber('/perception/sign_pose', MarkerArray, send_marker_transform)
broadcaster = tf2_ros.TransformBroadcaster()


rate = rospy.Rate(60)

while not rospy.is_shutdown():
	rate.sleep()
rospy.spin()

