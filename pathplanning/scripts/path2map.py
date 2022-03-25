import rospy
from crazyflie_driver.msg import Position
from tf.transformations import quaternion_from_euler
from a_star import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

path = "/home/maciejw/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/saal1.world.json"

def find_path(stx,sty,gox,goy):
    '''
    stx,sty: start point on map
    gox,goy: goal point on map
    px, py: path position
    '''
    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]
    
    mapp = Mapping(path, 0.05, 2)
    matrx = mapp.matrix
    range_of_map = matrx.shape
    horizonal = range_of_map[0]
    vertical = range_of_map[1]
    # set obstable positions
    matrx_indx = np.nonzero(matrx == 1) # represent the walls
    oy = matrx_indx[0].tolist()
    ox = matrx_indx[1].tolist()

    # path planning
    # start and goal position on mapping
    sx = (stx/mapp.step) + mapp.x_conv   # [m]
    sy = (sty/mapp.step) + mapp.y_conv  # [m]
    gx = (gox/mapp.step) + mapp.x_conv  # [m]
    gy = (goy/mapp.step) + mapp.y_conv # [m]
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)
    rx.reverse()
    ry.reverse()
    # path to follow
    px,py= [],[]
    px.append(stx)
    py.append(sty)
    for i in rx:
        temp=(i-mapp.x_conv)*mapp.step
        px.append(temp)
    for i in ry:
        temp=(i-mapp.y_conv)*mapp.step
        py.append(temp)
    px.append(gox)
    py.append(goy)

    view_path(px,py)
    
    return px,py

def view_path(pathx,pathy):
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
        pose.pose.position.z = 0.4
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
    stx=0.5
    sty=2.5
    gox=0.5
    goy=0.5
    while not rospy.is_shutdown():
        try:
            px,py = find_path(stx,sty,gox,goy)
        except rospy.ROSInterruptException:
            pass
        rate.sleep()