#!/usr/bin/env python
import rospy
import numpy as np
from aruco_msgs.msg import MarkerArray, Marker
import json
import copy

#from evaluate import load_dict


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
detected_ids = []


def get_marker_id(msg):
    #print(msg)
    global detected_ids
    detected_ids = []
    
    for marker in msg.markers:
        idd = marker.id
        detected_id = idd
        detected_ids.append(detected_id)
    #print(detected_ids)
        #print(idd)

def marker_id_list():
    with open('/home/maciejw/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/saal5.world.json') as f:
        world = json.load(f)

    signs = []
    for sign in world[u'roadsigns']:
    #    print(sign[u'sign'] + ', has id: ' + str(sign_dict[sign[u'sign'].encode('ascii','ignore')]))
        sign_name = sign[u'sign'].encode('ascii','ignore')
        signs.append(sign_dict[sign_name])

    return np.array(signs)



rospy.init_node('intruder')
rospy.Subscriber('/perception/sign_pose', MarkerArray, get_marker_id)

#broadcaster = tf2_ros.TransformBroadcaster()


rate = rospy.Rate(30)

signs = marker_id_list()
#signs = np.array([1])

print(signs)

intruder_detected = False
prev_intruder = None
count = 0

while not rospy.is_shutdown():
    #print(detected_id)
    if not intruder_detected:
        for detected_id in detected_ids:
            print(detected_id)
            if detected_id in signs:
                print('sign ' + str(detected_id) + 'is not an intruder')
                pass
            else:
                count += 1
                print('sign ' + str(detected_id) + 'is an intruder')
                #print('sign: ' + str(detected_id) + ' is not and intruder.')

            if count > 4:
                intruder_detected = True
                intruder = copy.deepcopy(detected_id)
   
    else:
        print('INTRUDER HAS BEEN DETECTED: ' + str(intruder))
        break
    rate.sleep()
rospy.spin()
