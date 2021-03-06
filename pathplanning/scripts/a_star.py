#!/usr/bin/env python
"""
A* grid planning
"""

import math
from mapping import Mapping
import numpy as np
import matplotlib.pyplot as plt
import time

show_animation = True
path = "/home/maciejw/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/saal4.world.json"

class AStarPlanner:

    def __init__(self, ox, oy, reso, rr):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [matrix m]
        oy: y position list of Obstacles [matrix m]
        reso: grid resolution 
        rr: robot radius
        """

        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, pind):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.pind = pind

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search
        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        nstart = self.Node(self.calc_xyindex(sx, self.minx),
                           self.calc_xyindex(sy, self.miny), 0.0, -1)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                          self.calc_xyindex(gy, self.miny), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal, open_set[o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.minx),
                         self.calc_grid_position(current.y, self.miny), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == ngoal.x and current.y == ngoal.y:
                # print("Find goal")
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)


                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(ngoal, closed_set)

        return rx, ry

    def calc_final_path(self, ngoal, closedset):
        # generate final course
        rx, ry = [self.calc_grid_position(ngoal.x, self.minx)], [
            self.calc_grid_position(ngoal.y, self.miny)]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            rx.append(self.calc_grid_position(n.x, self.minx))
            ry.append(self.calc_grid_position(n.y, self.miny))
            pind = n.pind

        return rx, ry, 

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, minp):
        pos = index * self.reso + minp
        return pos

    def calc_xyindex(self, position, min_pos):
        return round((position - min_pos) / self.reso)

    def calc_grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.minx)
        py = self.calc_grid_position(node.y, self.miny)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False
        # collision check
        if self.obmap[int(node.x)][int(node.y)]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.minx = round(min(ox))
        self.miny = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))
        # print("minx:", self.minx)
        # print("miny:", self.miny)
        # print("maxx:", self.maxx)
        # print("maxy:", self.maxy)

        self.xwidth = round((self.maxx - self.minx) / self.reso)
        self.ywidth = round((self.maxy - self.miny) / self.reso)
        # print("xwidth:", self.xwidth)
        # print("ywidth:", self.ywidth)

        # obstacle map generation
        self.obmap = [[False for i in range(int(self.ywidth))]
                      for i in range(int(self.xwidth))]
        for ix in range(int(self.xwidth)):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(int(self.ywidth)):
                y = self.calc_grid_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obmap[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

    #Ok, we can discuss it on zoom after the meeting., 
    # I will go for lunch now, see u then
def path_points(mapp, start, end, rx, ry):
    # start and end is the user input on map frame, which are start point and goal
    # rx, ry is the route on the matrix
    # px, py is the route on the map
    # pathx, pathy is the path for drone to fly

    px,py= [],[]
    px.append(start[0])
    py.append(start[1])
    for routex in rx:
        real=(routex + mapp.x_conv) * mapp.step
        px.append(real)
    for routey in ry:
        real=(routey + mapp.y_conv) * mapp.step
        py.append(real)
    px.append(end[0])
    py.append(end[1])
    # pathx,pathy = [], []
    # rx_filtered = []
    # ry_filtered = []
    # for i in range(rx):
    #     dy1 = ry[i+1] - ry[i]
    #     dx1 = rx[i+1] - ry[i]
    #     dy2 = ry[i+2] - ry[i+1]
    #     dx2 = rx[i+2] - ry[i+1] 

    #     if (dy1/dx1 != dy2/dy2):
    #         rx_filtered.append(rx[i+1])
    #         ry_filtered.append(ry[i+1])

    return px, py

def main():
    print("start!!")
    # saal3
    # start = [0.3, 0.15]
    # end = [1.1, 0.15]
    # flight trial
    # start = [0.5, -0.5]
    # end = [3, -2]
    # saal4
    start = [0.7, 0.7]
    end = [0.7, 2.1]

    t0=time.time()
    grid_size = 2.0  # configure it to needs
    robot_radius = 0.5  # drone radius
    
    mapp = Mapping(path, 0.1, 2)
    matrx = mapp.matrix

    # npmatrx = np.array()
    # print(horizonal)
    # print(vertical)

    # set obstable positions
    matrx_indx = np.nonzero(matrx == 1) # represent the walls
    oy = matrx_indx[0].tolist()
    ox = matrx_indx[1].tolist()

    # range_of_map = matrx.shape
    # horizonal = range_of_map[0]
    # vertical = range_of_map[1]
    # oy_old = matrx_indx[0].tolist()
    # ox_old = matrx_indx[1].tolist()
    # oy = [vertical-i for i in oy_old]
    # ox = [horizonal-i for i in ox_old]

    # start and goal position
    sx = (start[0]/mapp.step) - mapp.x_conv # [matrix]
    sy = (start[1]/mapp.step) - mapp.y_conv  # [matrix]
    gx = (end[0]/mapp.step) - mapp.x_conv # [matrix]
    gy = (end[1]/mapp.step) - mapp.y_conv # [matrix]
    
    # sx=3
    # sy=2
    # gx=10
    # gy=2
    # print(mapp.x_conv,mapp.y_conv)
    print("startpoint and endpoint:", sx,sy,gx,gy)
    # print(sx,sy,gx,gy)

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")
    t3=time.time()
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry= a_star.planning(sx, sy, gx, gy)
    rx.reverse()
    ry.reverse()
    # print(mapp.x_conv,mapp.y_conv)
    
    px,py=path_points(mapp,start,end,rx,ry)


    t1=time.time()
    total=t1-t3
    print("total time", total)

    if show_animation:  # pragma: no cover
        print(px)
        print(py)
        plt.plot(rx, ry, "-r")
        plt.show()
    #plt.show()

if __name__ == '__main__':
    main()
