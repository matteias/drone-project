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
    '''
    to find the goals on the map
    path_world is the path location of world map
    goal_arr is the array contains goals, in a order of[x, y, z, roll, pitch, yaw]
    '''
    pose_arr = []
    ori_arr = []
    goal_arr =[]
    margin = 0.4
    with open(path_world) as jfile:
        map_data = json.load(jfile)
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
                goal_arr[i][1] = goal_arr[i][1] + margin
            elif obj_arr[i][5] == 90:
                goal_arr[i][0] = goal_arr[i][0] - margin
            elif obj_arr[i][5] == 180:
                goal_arr[i][1] = goal_arr[i][1] - margin
            elif obj_arr[i][5] == -90:
                goal_arr[i][0] = goal_arr[i][0] + margin       
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
    pathx, pathy, pathz: final path position
    pathyaw: yaw angle for path
    '''
    grid_size = 2  # [m]
    robot_radius = 0.5  # [m]
    
    mapp = Mapping(path_world, 0.1, 2)
    matrx = mapp.matrix

    matrx_indx = np.nonzero(matrx == 1) # represent the walls
    oy = matrx_indx[0].tolist()
    ox = matrx_indx[1].tolist()

    # start and goal position0
    sx = (start[0]/mapp.step) - mapp.x_conv # [matrix]
    sy = (start[1]/mapp.step) - mapp.y_conv  # [matrix]
    gx = (end[0]/mapp.step) - mapp.x_conv # [matrix]
    gy = (end[1]/mapp.step) - mapp.y_conv # [matrix]

    # print("startpoint and endpoint:", sx,sy,gx,gy)
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
    px,py =path_points(mapp,start,end,rx,ry)
    pathx,pathy=path_simplify(px,py)

    pathz, pathyaw = [], []
    z_start = start[2]
    yaw_start = start[3]
    pathz.append(z_start)
    pathyaw.append(yaw_start)
    
    z_end = end[2]
    yaw_end = end[3]
    z=z_start
    yaw=yaw_start
    for i in range(len(pathx)-1):
        if z_start<z_end:
            if z<z_end:
                z+=0.05
            else:
                z=z_end
        elif z_start>z_end:
            if z>z_end:
                z-=0.05
            else:
                z=z_end
        else:
            z=z_end
        pathz.append(z)
        if yaw_start<yaw_end:
            if yaw<yaw_end:
                yaw+=45
            else:
                yaw=yaw_end
        elif yaw_start>yaw_end:
            if yaw>yaw_end:
                yaw-=45
            else:
                yaw=yaw_end
        else:
            yaw=yaw_end
        pathyaw.append(yaw)

    pathz.append(z_end)
    pathyaw.append(yaw_end)
    if show_animation:  # pragma: no cover
        # print(pathx)
        # print(pathy)
        print('Final path',(pathx,pathy,pathz,pathyaw))
        plt.plot(rx, ry, "-r")
        plt.show()
    
    return pathx, pathy, pathz, pathyaw

def path_simplify(pathx, pathy):
    """
    Test many nodes and find the longest possible direct path between them.
    """
    if len(pathx) <= 2:
        return pathx, pathy

    result_x=[]
    result_y=[]

    # print("Simplifying path:", [pathx,pathy])
    
    result_x.append(pathx[0])
    result_y.append(pathy[0])
    
    for i in range(len(pathx)-2):
       if (pathx[i+1]-pathx[i])*(pathy[i+2]-pathy[i+1]) != (pathx[i+2]-pathx[i+1])*(pathy[i+1]-pathy[i]):
           result_x.append(pathx[i+1])
           result_y.append(pathy[i+1])
    
    result_x.append(pathx[len(pathx)-1])
    result_y.append(pathy[len(pathx)-1])
    # print("Final path:", [result_x,result_y])
    return result_x, result_y

def view_path(pathx, pathy, pathz, pathyaw):
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
        pointz = pathz[i]
        pointyaw = pathyaw[i]
        pose = PoseStamped()
        pose.pose.position.x = pointx
        pose.pose.position.y = pointy
        pose.pose.position.z = pointz
        quaternion = quaternion_from_euler(0, 0, pointyaw)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        path.poses.append(pose)
    
    pub_path.publish(path)
    rospy.loginfo("Published {} waypoints.".format(len(path.poses))) 

def exe_path(stx, sty, stz, styaw, gox, goy, goz, goyaw):
    '''
    stx,sty: start point on map
    gox,goy: goal point on map
    '''

    # Find Path
    pathx, pathy, pathz, pathyaw = find_path([stx,sty,stz,styaw], [gox,goy,goz,goyaw], path_world)

    view_path(pathx, pathy, pathz, pathyaw)
    # Execute Path
    goal = PoseStamped()
    goal.header.frame_id = 'map'

    rate = rospy.Rate(1) 
    if not rospy.is_shutdown():
        for i in range(len(pathx)):
            print(i+1)
            goal.pose.position.x = pathx[i]
            goal.pose.position.y = pathy[i]
            goal.pose.position.z = pathz[i]
            [goal.pose.orientation.x, 
 	         goal.pose.orientation.y,
 	         goal.pose.orientation.z,
 	         goal.pose.orientation.w ] = quaternion_from_euler( math.radians(0),
                                                                math.radians(0),
                                                                pathyaw[i] )
            goal.header.stamp = rospy.Time.now()
            pub_goal.publish(goal)
            rate.sleep()

rospy.init_node('path2map', anonymous=True)
pub_goal  = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=2)
rate = rospy.Rate(10)

if __name__ == '__main__':
    points_x, points_y, points_z, points_yaw = find_goal(path_world)
    try:
        for i in range(len(points_x)-1):
            stx = points_x[i]
            sty = points_y[i]
            stz = points_z[i]
            styaw = points_yaw[i]
            gox = points_x[i+1]
            goy = points_y[i+1]
            goz = points_z[i+1]
            goyaw = points_yaw[i+1]
            
            print('--------------------------------------------')
            print('Setting start:')
            print(stx, sty, stz, styaw)
            print('and goal:')
            print(gox, goy, goz, goyaw)
            exe_path(stx, sty, stz, styaw, gox, goy, goz, goyaw)
            print('Path executed')
            rospy.sleep(1)
    except rospy.ROSInterruptException:
        pass
    