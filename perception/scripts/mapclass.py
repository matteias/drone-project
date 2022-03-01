import numpy as np
import matplotlib.pyplot as plt
class Map:
    def __init__(self,maparray,x0,y0,xL,yL):
        self.OCG = maparray
        self.x0 = x0
        self.y0 = y0
        self.nx = self.OCG.shape[0]
        self.ny = self.OCG.shape[1]
        self.dx = xL/self.nx
        self.dy = yL/self.ny
        self.getConvexCorners()
        self.getCostGraph()
        print(self.getPoint(3,3))
        print(self.getPoint(2,2)*0.5 + self.getPoint(3,3)*0.5)
        print(self.OCG)
        self.plotmap()
        l0 = np.array([0,.1])
        l1 = np.array([4,1.3])
        print(self.checkCollision(l0,l1))

        l0ind = self.getindex(l0[0],l0[1])
        l1ind = self.getindex(l1[0],l1[1])
        self.drawcorners()
        self.plotline(l0ind,l1ind)
        plt.show()
    def plotmap(self):
        plt.imshow(self.OCG)

    def plotline(self,l0,l1):
        x = np.array([l0[0],l1[0]])
        y = np.array([l0[1],l1[1]])
        plt.plot(x,y)

    def drawcorners(self):
        i = []
        j = []
        for corner in self.corners:
            indexPoint = self.getindex(corner[0],corner[1])
            i.append(indexPoint[0])
            j.append(indexPoint[1])
        plt.scatter(i,j)
    def getConvexCorners(self):
        #y is i x is j
        self.corners = []
        weight = 0.51
        for i in range(1,self.ny-1):
            for j in range(1,self.nx-1):
                occupied = self.OCG[i,j] > 0.5
                if occupied:
                    lowerLeft = self.OCG[i+1,j] < 0.5 and self.OCG[i+1,j-1] < 0.5 and self.OCG[i,j-1] < 0.5
                    upperLeft = self.OCG[i-1,j] < 0.5 and self.OCG[i-1,j-1] < 0.5 and self.OCG[i,j-1] < 0.5
                    upperRight = self.OCG[i-1,j] < 0.5 and self.OCG[i-1,j+1] < 0.5 and self.OCG[i,j+1] < 0.5
                    lowerRight = self.OCG[i+1,j] < 0.5 and self.OCG[i+1,j+1] < 0.5 and self.OCG[i,j+1] < 0.5

                    if lowerLeft:
                        point = (1.0-weight)*self.getPoint(i,j) + weight*self.getPoint(i+1,j-1)
                        self.corners.append(point)
                    if upperLeft:
                        point = (1.0-weight)*self.getPoint(i,j) + weight*self.getPoint(i-1,j-1)
                        self.corners.append(point)
                    if upperRight:
                        point = (1.0-weight)*self.getPoint(i,j) + weight*self.getPoint(i-1,j+1)
                        self.corners.append(point)
                    if lowerRight:
                        point = (1.0-weight)*self.getPoint(i,j) + weight*self.getPoint(i+1,j+1)
                        self.corners.append(point)
        print(self.corners)
    def getCostGraph(self):
        N=len(self.corners)
        self.CostArray = np.zeros((N+2,N+2))
        for i in range(1,N-1):
            for j in range(1,N-1):
                #Calculate cost from corner i to corner j
                pass
    def calculateStartGoalCost(self,start,goal):
        #Fill in the edges of the cost graph
        pass
    def getPath(self,start,goal):
        self.calculateStartGoalCost(self,start,goal)
        path = []
        return path
    def getx(self,j):
        return j*self.dx + self.x0 + self.dx*0.5
    def gety(self,i):
        return i*self.dy + self.y0 + self.dy*0.5
    def getPoint(self,j,i):
        return np.array([self.getx(j),self.gety(i)])
    def geti(self,y):
        return (y - self.y0 - self.dy*0.5)/self.dy
    def getj(self,x):
        return (x - self.x0 - self.dx*0.5)/self.dx
    def getindex(self,x,y):
        i = self.geti(y)
        j = self.getj(x)
        return np.array([i,j])
    def checkCollisionCell(self,start,end,i,j):
        p1 = np.array([self.getx(i) - self.dx*0.5, self.gety(j) - self.dy*0.5, 0])
        p2 = np.array([self.getx(i) - self.dx*0.5, self.gety(j) + self.dy*0.5, 0])
        p3 = np.array([self.getx(i) + self.dx*0.5, self.gety(j) + self.dy*0.5, 0])
        p4 = np.array([self.getx(i) + self.dx*0.5, self.gety(j) - self.dy*0.5, 0])
        l0 = start
        l1 = end
        if self.getLineCollisionPoint(p1,p2,l0,l1) or self.getLineCollisionPoint(p2,p3,l0,l1) or self.getLineCollisionPoint(p3,p4,l0,l1) or self.getLineCollisionPoint(p4,p1,l0,l1):
            return True
        else:
            return False

    def getLineCollisionPoint(self,p0,p1,l0,l1):
        p0x = p0[0]
        p0y = p0[1]
        p1x = p1[0]
        p1y = p1[1]

        l0x = l0[0]
        l0y = l0[1]
        l1x = l1[0]
        l1y = l1[1]
        LHS = np.array([[p1x-p0x, l0x-l1x],[p1y-p0y, l0y-l1y]])
        RHS = np.array([ [ l0[0]-p0[0] ],[ l0[1]-p0[1] ]])
        try:
            st = np.linalg.solve(LHS,RHS)
            if abs(st[0]-0.5) < 0.5 and abs(st[1]-0.5) < 0.5:
                return True
        except:
            pass
        return False

    def checkCollision(self,start,end):
        for i in range(0,self.ny):
            for j in range(0,self.nx):

                if self.OCG[i,j] > 0.5 and self.checkCollisionCell(start,end,i,j):
                    return True
        return False

    def getLinesFromIndex(self,i,j):
        #Gives left,upper,right and lower lines in that order in homogenous coordinates
        weight = 0.5
        p1 = np.array([self.getx(j) - self.dx*0.5, self.gety(i) - self.dy*0.5, 0])
        p2 = np.array([self.getx(j) - self.dx*0.5, self.gety(i) + self.dy*0.5, 0])
        p3 = np.array([self.getx(j) + self.dx*0.5, self.gety(i) + self.dy*0.5, 0])
        p4 = np.array([self.getx(j) + self.dx*0.5, self.gety(i) - self.dy*0.5, 0])
        L1 = np.cross(p1,p2)
        L2 = np.cross(p2,p3)
        L3 = np.cross(p3,p4)
        L4 = np.cross(p4,p1)
        return [L1,L2,L3,L4]

testarray = np.array([[0,0,1,0],[0,1,1,0],[0,1,1,0],[0,0,0,0]])
map = Map(testarray,0,0,4,4)
