#!/usr/bin/env python

import rospy
from std_msgs.msg import String
#from crazyflie_driver.msg import Position
from a_star import *

def find_path():
    sx = 35.0  # [m]
    sy = 25.0  # [m]
    gx = 25.0  # [m]
    gy = 45.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]

    mapp = Mapping(path, 0.1, 2)
    matrx = mapp.matrix
    range_of_map = matrx.shape
    horizonal = range_of_map[0]
    vertical = range_of_map[1]
    # print(matrx.shape)
    # print(horizonal)
    # print(vertical)

    # set obstable positions
    matrx_indx = np.nonzero(matrx == 1) # represent the walls
    oy_old = matrx_indx[0].tolist()
    ox_old = matrx_indx[1].tolist()
    oy = [vertical-i for i in oy_old]
    ox = [horizonal-i for i in ox_old]
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)
    rx.reverse()
    ry.reverse()
    return rx,ry

def publish_path():
    # pathx = [74.0, 76.0, 78.0, 80.0, 82.0, 84.0, 86.0, 88.0, 90.0, 92.0, 94.0, 96.0, 98.0, 100.0, 102.0, 104.0, 106.0, 108.0, 110.0, 112.0, 114.0, 116.0, 118.0, 120.0]
    # pathy = [100.0, 100.0, 100.0, 102.0, 102.0, 104.0, 106.0, 106.0, 106.0, 108.0, 108.0, 110.0, 110.0, 110.0, 112.0, 112.0, 114.0, 114.0, 116.0, 116.0, 118.0, 118.0, 118.0, 120.0]
    rospy.init_node('path_following', anonymous=True)
    rx,ry = find_path()
    # print(rx)
    # print(ry)
    pathx = [c/100 for c in rx]
    pathy = [c/100 for c in ry]
    pathz = [0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4]
    pathyaw = [0]*11
    cmd = Position()

    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = '10' # a simple random number


    pub = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        for i in range(11):
            cmd.x = pathx[i]
            cmd.y = pathy[i]
            cmd.z = pathz[i]
            cmd.yaw = pathyaw[i]
            pub.publish(cmd)
            rate.sleep()
        for j in range(11):
            cmd.x = pathx[10-j]
            cmd.y = pathy[10-j]
            cmd.z = pathz[10-j]
            cmd.yaw = pathyaw[10-j]
            pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass