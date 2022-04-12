#!/usr/bin/env python
import rospy
import json
from aruco_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseWithCovariance, Point, Quaternion, TransformStamped, Vector3
import tf2_ros
from tf.transformations import quaternion_from_euler
import math


sign_dict = {
    "no_bicycle": 0,
    "airport": 1,
    "dangerous_curve_left": 2,
    "dangerous_curve_right": 3,
    "follow_left": 4,
    "follow_right": 5,
    "junction": 6,
    "no_heavy_truck": 7,
    "no_parking": 8,
    "no_stopping_and_parking": 9,
    "residential": 10,
    "road_narrows_from_left": 11,
    "road_narrows_from_right": 12,
    "roundabout_warning": 13,
    "stop": 14,
}

# dynamic transform for deteceted signs
def send_marker_transform(msg):
    for marker in msg.markers:
        t = TransformStamped()
        t.header.stamp = marker.header.stamp
        if marker.id != 0:
            t.header.frame_id = 'cf1/camera_link'
            t.child_frame_id = 'perception/detected'+str(marker.id)
            t.transform.translation.x = marker.pose.pose.position.x
            t.transform.translation.y = marker.pose.pose.position.y
            t.transform.translation.z = marker.pose.pose.position.z
            t.transform.rotation.x = marker.pose.pose.orientation.x
            t.transform.rotation.y = marker.pose.pose.orientation.y
            t.transform.rotation.z = marker.pose.pose.orientation.z
            t.transform.rotation.w = marker.pose.pose.orientation.w
            broadcaster.sendTransform(t)

# static transform for roadsigns in map
def transform_from_sign(m):
    print(m)
    t = TransformStamped()
    t.header.frame_id = 'map'
    id = sign_dict[m[u'sign'].encode('ascii','ignore')]
    t.child_frame_id = 'perception/sign' + str(id)
    t.transform.translation = Vector3(*m[u'pose'][u'position'])
    roll, pitch, yaw = m[u'pose'][u'orientation']
    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) = quaternion_from_euler(math.radians(roll),
                                                     math.radians(pitch),
                                                     math.radians(yaw))
    return t


rospy.init_node('sign_publisher')
rospy.Subscriber('/perception/sign_pose', MarkerArray, send_marker_transform)
broadcaster = tf2_ros.TransformBroadcaster()


# Load world JSON
with open('/home/matte/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/tutorial_1.world.json') as f:
        world = json.load(f)

transforms = [transform_from_sign(m) for m in world[u'roadsigns']]

# Publish these transforms statically forever
static_broadcaster = tf2_ros.StaticTransformBroadcaster()
static_broadcaster.sendTransform(transforms)


rate = rospy.Rate(30)

while not rospy.is_shutdown():
	rate.sleep()
rospy.spin()
