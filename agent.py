import math
class basicAgent:
    def __init__(self,vGain,thetaGain,desiredV):
        self.vGain=vGain
        self.thetaGain=thetaGain
        self.desiredV=desiredV

    def setVehicle(self,vehicle):
        self.vehicle=vehicle
    
    def getPos(self):
        return self.vehicle.sensor.getSelfPos()

    def getDis(self):
        return self.vehicle.sensor.getCordPos()[0]

    def getVelocity(self):
        return self.vehicle.sensor.getVelocity()

    def getAngle(self):
        return self.vehicle.sensor.getCordAngle()
    
    def doControl(self):
        acceleration=self.vGain*(self.desiredV-self.getVelocity())-math.cos(self.getAngle())
        #steerV=self.thetaGain*self.getAngle()
        steerV=-self.thetaGain*self.getAngle()
        return [acceleration,steerV]
                
