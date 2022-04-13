#!/usr/bin/env python

import rospy
from crazyflie_driver.msg import Position
from tf.transformations import quaternion_from_euler
from a_star import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# path_world = "/home/maciejw/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/saal3.world.json"

def find_path(start, end, path_world):
    '''
    start: start point on map
    end: goal point on map
    px, py: path position
    '''
    grid_size = 2  # [m]
    robot_radius = 0.5  # [m]
    
    mapp = Mapping(path_world, 0.1, 2)
    matrx = mapp.matrix

    matrx_indx = np.nonzero(matrx == 1) # represent the walls
    oy = matrx_indx[0].tolist()
    ox = matrx_indx[1].tolist()

    # start and goal position
    sx = (start[0]/mapp.step) - mapp.x_conv # [matrix]
    sy = (start[1]/mapp.step) - mapp.y_conv  # [matrix]
    gx = (end[0]/mapp.step) - mapp.x_conv # [matrix]
    gy = (end[1]/mapp.step) - mapp.y_conv # [matrix]

    print("startpoint and endpoint:", sx,sy,gx,gy)
    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)
    rx.reverse()
    ry.reverse()

    # path to follow
    px,py=path_points(mapp,start,end,rx,ry)
    if show_animation:  # pragma: no cover
        print(px)
        print(py)
        plt.plot(rx, ry, "-r")
        plt.show()
    view_path(px,py)
    
    return px,py

def view_path(pathx, pathy):
    '''
    Publish the ROS path msgs containing the way point
    '''
    pub_path  = rospy.Publisher('path', Path, queue_size=2)
    
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()
    for i in range(len(pathx)):
        pointx = pathx[i]
        pointy = pathy[i]
        pose = PoseStamped()
        pose.pose.position.x = pointx
        pose.pose.position.y = pointy
        pose.pose.position.z = 0.3
        quaternion = quaternion_from_euler(0, 0, 0)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        path.poses.append(pose)
    
    pub_path.publish(path)
    rospy.loginfo("Published {} waypoints.".format(len(path.poses))) 

rospy.init_node('path_planner', anonymous=True)
rate = rospy.Rate(10)

if __name__ == '__main__':
    start = [0.3, 0.15]
    end = [1.1, 0.15]
    while not rospy.is_shutdown():
        try:
            px,py = find_path(start, end)
        except rospy.ROSInterruptException:
            pass
        rate.sleep()