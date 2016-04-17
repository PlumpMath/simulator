import numpy as np
import math

class basicSensor:
    
    def __init__(self,game):
        self.game=game
        self.Line=self.game.segLine
        self.leftPoints=self.game.road.leftPoints
        self.rightPoints=self.game.road.rightPoints
        self.isAlign=False
        self.cordNum=0
        self.bias=0
        
    def setVehicle(self,vehicle):
        self.vehicle=vehicle

    # get direction vector in world cordination        
    def getSelfAngle(self):
        direction=self.vehicle.getDirection()
        return np.array([direction[0],direction[1]])

    # global self vehicle position in world cordination
    def getSelfPos(self):
        return np.array([self.vehicle.getPos()[0],self.vehicle.getPos()[1]])

    def isInPoly(self,num,point):
        pos=self.getSelfPos()
        A=self.leftPoints[num]
        B=self.leftPoints[num+1]
        D=self.rightPoints[num]
        C=self.rightPoints[num+1]
        M=point
        if np.cross(B-A,M-A)<0 and np.cross(C-B,M-B)<0 and np.cross(D-C,M-C)<0 and np.cross(A-D,M-D)<0:
            return True
        else:
            return False
        
    def getDisBiasFromLine(self,point,num):
        y1=self.Line[num][1]
        y2=self.Line[num+1][1]
        x1=self.Line[num][0]
        x2=self.Line[num+1][0]
        A=y1-y2
        B=x2-x1
        C=y2*(x1-x2)+x2*(y2-y1)
        x0=point[0]
        y0=point[1]
        d=(A*x0+B*y0+C)/math.sqrt(A*A+B*B)
        k=B/A
        x=(B*k*x0-C-B*y0)/(A+B*k)
        y=y0+k*(x-x0)
        bias=np.linalg.norm(np.array([x,y])-self.Line[num])
        return [-d,bias]

    # get direction vector of the centerline in current position
    def getLineAngle(self):
        return self.Line[self.cordNum+1]-self.Line[self.cordNum]

    # get angle in road cordination (left positive)
    def getCordAngle(self):
        lineAngle=self.getLineAngle()
        selfAngle=self.getSelfAngle()
        angle=math.acos(np.dot(lineAngle,selfAngle)/(np.linalg.norm(lineAngle)*np.linalg.norm(selfAngle)))
        if np.cross(lineAngle,selfAngle)<=0:
            angle=-angle
        return angle

    # get position in road cordination [distanceFromCenterline(left positive), distanceFromStart]
    def getCordPos(self):
        pos=self.getSelfPos()
        prevNum=self.cordNum
        for i in range(max(self.cordNum-2,0),self.cordNum+2):
            if self.isInPoly(i,pos):
                self.cordNum=i
                break
        for j in range(prevNum,self.cordNum):
            self.bias=self.bias+np.linalg.norm(self.Line[j+1]-self.Line[j])
        disBias=self.getDisBiasFromLine(pos,self.cordNum)         
        bias=self.bias+disBias[1]
        return np.array([-disBias[0],bias])

    # get self vehicle velocity value
    def getVelocity(self):
        return self.vehicle.getVelocity()

    def align(self):
        if self.isAlign==False:
            pos=self.getSelfPos()
            for i in range(max(self.vehicle.initNum-10,0),self.vehicle.initNum+10):
                if self.isInPoly(i,pos):
                    self.isAlign==True
                    self.cordNum=i
                    print self.cordNum
                    for j in range(0,i):
                        self.bias=self.bias+np.linalg.norm(self.Line[j+1]-self.Line[j])
                    break

    # get a piece of centerline points
    def getLineInRange(self,prevDis,posDis):
        prevNum=self.cordNum-math.ceil(prevDis/self.game.precision)
        posNum=self.cordNum+math.ceil(posDis/self.game.precision)
        return self.Line[self.cordNum-prevNum:self.cordNum+posNum+1]
            
            
        
        
