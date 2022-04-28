#!/usr/bin/env python

import json
import rospy
from crazyflie_driver.msg import Position
from tf.transformations import quaternion_from_euler
from a_star import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import copy

path_world = "/home/maciejw/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/flight_trial_2.world.json"

def find_goal(path_world):
    pose_arr = []
    ori_arr = []
    goal_arr =[]
    with open(path_world) as jfile:
        map_data = json.load(jfile)
        map_start = map_data["airspace"]["min"]
        map_end = map_data["airspace"]["max"]
    for i in map_data["markers"]:
        marker_pose = i["pose"]["position"]
        marker_ori = i["pose"]["orientation"]
        pose_arr.append(marker_pose)
        ori_arr.append(marker_ori)
    for i in map_data["roadsigns"]:
        sign_pose = i["pose"]["position"]
        sign_ori = i["pose"]["orientation"]
        pose_arr.append(sign_pose)
        ori_arr.append(sign_ori)
    pose_arr=np.array(pose_arr)
    ori_arr=np.array(ori_arr)

    obj_arr=np.append(pose_arr,ori_arr,axis=1)
    # sort the position based on y axis
    ind = np.argsort(obj_arr[:,0])
    obj_arr=obj_arr[ind]
    goal_arr = copy.deepcopy(obj_arr)

    for i in range(obj_arr.shape[0]):
        if obj_arr[i][2]!=0:
            if obj_arr[i][5] == 0:       
                goal_arr[i][1] = goal_arr[i][1] + 0.25
            elif obj_arr[i][5] == 90:
                goal_arr[i][0] = goal_arr[i][0] - 0.25
            elif obj_arr[i][5] == 180:
                goal_arr[i][1] = goal_arr[i][1] - 0.25
            elif obj_arr[i][5] == -90:
                goal_arr[i][0] = goal_arr[i][0] + 0.25       
        else:
            print("something went wrong from world map",i)
    goal_arr[:,5] += 180
    goalx = goal_arr[:,0]
    goaly = goal_arr[:,1]
    goalz = goal_arr[:,2]
    goalyaw = goal_arr[:,5]
    return goalx, goaly, goalz, goalyaw
    
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
    pathx,pathy=path_simplify(matrx,)
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

def path_simplify(grid, pathx,pathy):
    """
    Test many nodes and find the longest possible direct path between them.
    """
    path=[]
    for i in range(len(pathx)):
        p=[pathx[i],pathy[i]]
        path.append(p)
    path = np.array(path)
    if len(path) <= 2:
        return path
    print("Simplifying path:", [path, path])
    start_idx = 0
    end_idx = len(path) - 1
    result_path = [path[0]]
    while start_idx < end_idx:
        start = path[start_idx]
        end = path[end_idx]
        min_height = min(start[2], end[2])
        cells = bresenham(start, end)

        has_obs = False
        for n, e in cells:
            if grid[n, e] >= min_height:
                has_obs = True
                break

        if has_obs:
            end_idx -= 1
            if end_idx == start_idx:
                print("No direct path! {}".format(path[start_idx]))
        else:
            result_path.append(end)
            start_idx = end_idx
            end_idx = len(path) - 1
    if result_path[-1] != path[-1]:
        result_path.append(path[-1])

    print("Final path:", result_path)
    return result_path

def bresenham(start, end):
    n1, e1 = start[:2]
    n2, e2 = end[:2]

    if abs(e2 - e1) < 1e-5:
        return [(n, e1) for n in range(min(n1, n2), max(n1, n2) + 1)]

    slope = (n2 - n1) / (e2 - e1)

    if e1 < e2:
        n, e = n1, e1
        ne, ee = n2, e2
    else:
        n, e = n2, e2
        ne, ee = n1, e1

    cells = []

    f = n
    if slope >= 0:
        while e <= ee and n <= ne:
            cells.append((n, e))
            f_new = f + slope
            if f_new > n + 1:
                n += 1
            else:
                e += 1
                f = f_new
    else:
        while e <= ee and n >= ne:
            cells.append((n, e))
            f_new = f + slope
            if f_new < n - 1:
                n -= 1
            else:
                e += 1
                f = f_new

    return cells
rospy.init_node('path_planner', anonymous=True)
rate = rospy.Rate(10)

if __name__ == '__main__':
    start = [0.8, 0.15]
    end = [4, 3.5]
    try:
        px,py = find_path(start, end, path_world)
    except rospy.ROSInterruptException:
        pass
    