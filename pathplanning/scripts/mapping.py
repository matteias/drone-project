#!/usr/bin/env python

import json
import matplotlib.pyplot as plt
import numpy as np
import sys
import rospy
from std_srvs.srv import SetBool
from rospy.numpy_msg import numpy_msg

# todo fix the expansion, centralize the origin when expansion is applied


# zm is only a placeholder

class Mapping:

    def __init__(self, json_file_dir, step_size, inflation, expansion=0):
        # json_file_dir is the directory where the json file is located
        # step size is how small each grid will be. 0.1 is 10 cm, which is what I used
        # inflation is how big you want the walls to be, needs fixing
        # expansion does not work yet, it shifts the map

        self.jfile = json_file_dir
        self.step = step_size
        self.infl = inflation

        with open(self.jfile) as jfile:
            self.map_data = json.load(jfile)
            map_end, map_start = self.airspace() # map_end map_start is the max and min of airspaces, where map starts and ends
            air_end, air_start = self.airspace()
            if expansion > 0:
                map_start, map_end = self.expand(map_start, map_end, expansion) # need fixing
            
            # conv is how we convert the real coordinates to matrix map
            # map_start is the new origin of matrix map, so each coordinate should be converted
            # x_matrix = x/step_size - x_conv
            self.x_conv = (map_start[0])/step_size
            self.y_conv = (map_start[1])/step_size
            # ms = [map_start[0]+self.x_conv, map_start[1]+self.y_conv]
            # me = [map_end[0]+self.x_conv, map_end[1]+self.y_conv]
            self.matrix = self.clean_map(map_start, map_end, 0, self.step)

            self.matrix[:,0:inflation] = 1
            self.matrix[:,-inflation:] = 1
            self.matrix[0:inflation,:] = 1
            self.matrix[-inflation:,:] = 1
            for wall in self.map_data["walls"]:
                wall_start = wall["plane"]["start"]
                wall_stop = wall["plane"]["stop"]
                points = self.line(wall_start[:2], wall_stop[:2])
                self.add_objects(points, wall=True)

            self.objects(markers=True)
            self.objects(roadsigns=True)


    def airspace(self):
        # Extracting the boundries for the airspace
        max_coords = self.map_data["airspace"]["max"]
        min_coords = self.map_data["airspace"]["min"]
        return max_coords, min_coords

    def expand(self, start, end, exp):
        exp_start = (start[1] + exp, start[0] + exp)
        exp_end = (end[1] + exp, end[0] + exp)
        return exp_start, exp_end

    def objects(self, markers=False, roadsigns=False):
        # Adding objects to the map matrix
        if markers == True:
            size = self.map_data["marker_size"]
        elif roadsigns == True:
            size = self.map_data["roadsign_size"]
        else:
            print("No marker or roadSign!")
            return 0
        
        points = []
        
        if markers == True:    
            for marker in self.map_data["markers"]:
                (xm, ym, zm) = marker["pose"]["position"]
                point = (int(xm/self.step), int(ym/self.step))
                points.append(point)
            self.add_objects(points, marker=True)

        elif roadsigns == True:
            for marker in self.map_data["roadsigns"]:
                (xm, ym, zm) = marker["pose"]["position"]
                point = (int(xm/self.step), int(ym/self.step))
                points.append(point)
            self.add_objects(points, roadsign=True)

        else:
            # Raise error msg maybe?
            return 0
        

    def clean_map(self, start, end, expansion, step_size):
        # Generating a zero matrix for the entire map + some extending it a bit
        # since markers could be outside the airspace
        x = (end[0] - start[0] + expansion)/step_size
        y = (end[1] - start[1] + expansion)/step_size
        map_matrix = np.zeros((int(x), int(y)))
        return map_matrix
       
    def inflate_walls(self, xidx, yidx):
        # inflating the walls so they appear larger in the map
        yidx = int(yidx)
        xidx = int(xidx)
        xmin = xidx - self.infl
        xmax = xidx + self.infl
        ymin = yidx - self.infl
        ymax = yidx + self.infl
        xmin = max(0,xmin)
        ymin = max(0,ymin)
        xmax = min(self.matrix.shape[0],xmax)
        ymax = min(self.matrix.shape[1],ymax) 
        self.matrix[xmin:xmax,ymin:ymax] = 1
        
    
    def line(self, start, end):
        """
        Code of honor: Algorithm from http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm#Python
        """
        x1, y1 = start
        x2, y2 = end
        x1 = x1/self.step
        x2 = x2/self.step
        y1 = y1/self.step
        y2 = y2/self.step
        dx = x2 - x1
        dy = y2 - y1
        
        is_steep = abs(dy) > abs(dx)
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
        
        dx = x2 - x1
        dy = y2 - y1

        error = int(dx/2)
        ystep = 1 if y1 < y2 else -1

        y = y1
        points = []

        for x in range(int(x1), int(x2) + 1):
            coord = (y, x) if is_steep else(x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
        
        if swapped:
            points.reverse()
        
        return points
        
    def PixelIsInsideMap(self, p_shift):
        return p_shift[0] >= 0 and p_shift[1] >= 0 and p_shift[0] < self.matrix.shape[0] and p_shift[1] < self.matrix.shape[1]

    def add_objects(self, points, wall=False, marker=False, roadsign=False):
        # adding the assigned objects into the map matrix
        # points is the real coordinates divided by stepsize
        for p in points:

            xidx = -self.x_conv + p[0]
            yidx = -self.y_conv + p[1]
            
            p_shift = (min(int(xidx), self.matrix.shape[0]-1), min(int(yidx), self.matrix.shape[1]-1))
            if self.PixelIsInsideMap(p_shift):
                if wall == True:
                    self.matrix[p_shift] = 1
                    self.inflate_walls(xidx, yidx)
                elif marker == True:
                    self.matrix[p_shift] = 4
                elif roadsign == True:
                    self.matrix[p_shift] = 5
            else:
                print("Trying to place something outside map at: " +str(p_shift))

    
    #def wall_values(self):
        #pass

    def object_poses(self):
        # Create a list of lists containing the poses of the markers and signs.
        # Returns two lists with tuples. First element in the tuple is the marker id or roadsign name
        # Second element in the list is a list with the pose. [x y z angles] 
        objects = {}

        markers = []
        signs = []

        for object in self.map_data["markers"]:
            # Place holder for name or ID if needed.
            name = object["id"]
            pose = object["pose"]["position"] + object["pose"]["orientation"]
            
            # objects[name] = pose

            markers.append((name, pose))

        for object in self.map_data["roadsigns"]:
            # Place holder for name or ID if needed.
            name = object["sign"]
            pose = object["pose"]["position"] + object["pose"]["orientation"]
            
            objects[name] = pose

            signs.append((name, pose))
        
        return markers, signs, objects

