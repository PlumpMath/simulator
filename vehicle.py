from panda3d.bullet import BulletVehicle
from panda3d.bullet import ZUp
from panda3d.bullet import BulletBoxShape
from panda3d.core import Vec3
from panda3d.core import Vec4
from panda3d.core import TransformState
from panda3d.core import Point3
from panda3d.bullet import BulletRigidBodyNode
from direct.showbase.InputStateGlobal import inputState
import math
import numpy

def setChassis(game,position,velocity):
  shape = BulletBoxShape(Vec3(0.6, 1.4, 0.5))
  ts = TransformState.makePos(Point3(0, 0, 0.5))

  np = game.worldNP.attachNewNode(BulletRigidBodyNode('Vehicle'))
  np.node().addShape(shape, ts)
  np.setPos(position)
  np.node().setLinearVelocity(Vec3(0,velocity,0))
  np.node().setMass(1000.0)
  np.node().setDeactivationEnabled(False)

  game.world.attachRigidBody(np.node())
  return np

class basicVehicle(BulletVehicle):
  def __init__(self,game,pos,vel):
    self.initCordPos=numpy.array([pos[0],pos[1]])
    line=game.segLine
    lineLen=0
    prevPoint=line[0]
    self.initNum=0
    for point in line:
      norm=numpy.linalg.norm(point-prevPoint)
      if pos[0]>=lineLen and pos[0]<lineLen+norm:
        ldirec=(point-prevPoint)/norm
        rdirec=numpy.array([ldirec[1],-ldirec[0]])
        post=prevPoint+(pos[0]-lineLen)*ldirec+rdirec*pos[1]
        self.initPos=Vec3(post[0],post[1],pos[2])
        break
      lineLen=lineLen+norm
      prevPoint=point
      self.initNum=self.initNum+1
    chassis=setChassis(game,self.initPos,vel)
    BulletVehicle.__init__(self,game.world,chassis.node())
    self.setCoordinateSystem(ZUp)
    game.world.attachVehicle(self)
    self.yugoNP = loader.loadModel('models/yugo/yugo.egg')
    self.yugoNP.reparentTo(chassis)

    # Right front wheel
    np = loader.loadModel('models/yugo/yugotireR.egg')
    np.reparentTo(game.worldNP)
    self.addWheel(Point3( 0.70,  1.05, 0.3), True, np)

    # Left front wheel
    np = loader.loadModel('models/yugo/yugotireL.egg')
    np.reparentTo(game.worldNP)
    self.addWheel(Point3(-0.70,  1.05, 0.3), True, np)

    # Right rear wheel
    np = loader.loadModel('models/yugo/yugotireR.egg')
    np.reparentTo(game.worldNP)
    self.addWheel(Point3( 0.70, -1.05, 0.3), False, np)

    # Left rear wheel
    np = loader.loadModel('models/yugo/yugotireL.egg')
    np.reparentTo(game.worldNP)
    self.addWheel(Point3(-0.70, -1.05, 0.3), False, np)

    # Steering info
    self.steering = 0.0            # degree
    self.steeringClamp = 45.0      # degree
    self.steeringIncrement = 10.0  # degree per second

  def addWheel(self, pos, front, np):
    wheel = self.createWheel()

    wheel.setNode(np.node())
    wheel.setChassisConnectionPointCs(pos)
    wheel.setFrontWheel(front)

    wheel.setWheelDirectionCs(Vec3(0, 0, -1))
    wheel.setWheelAxleCs(Vec3(1, 0, 0))
    wheel.setWheelRadius(0.25)
    wheel.setMaxSuspensionTravelCm(40.0)

    wheel.setSuspensionStiffness(40.0)
    wheel.setWheelsDampingRelaxation(2.3)
    wheel.setWheelsDampingCompression(4.4)
    wheel.setFrictionSlip(100.0);
    wheel.setRollInfluence(0.1)
    
  def processInput(self, dt, up, back, left, right, brake):
    engineForce = 0.0
    brakeForce = 0.0
    #direction=self.getForwardVector()
    if inputState.isSet(up):
      engineForce = 2000.0
      brakeForce = 0.0
    
    if inputState.isSet(back):
      engineForce = -1000.0
      brakeForce = 0.0
    
    if inputState.isSet(brake):
      engineForce = 0.0
      brakeForce = 1000.0
      
    if inputState.isSet(left):
      self.steering += dt * self.steeringIncrement
      self.steering = min(self.steering, self.steeringClamp)
    elif inputState.isSet(right):
      self.steering -= dt * self.steeringIncrement
      self.steering = max(self.steering, -self.steeringClamp)
    else:
      self.steering=self.steering*0.7

    # Apply steering to front wheels
    self.setSteeringValue(self.steering, 0)
    self.setSteeringValue(self.steering, 1)

    # Apply engine and brake to rear wheels
    self.applyEngineForce(engineForce, 2)
    self.applyEngineForce(engineForce, 3)
    self.setBrake(brakeForce, 2)
    self.setBrake(brakeForce, 3)

  def getPos(self):
    return self.getChassis().getTransform().getPos()

  def getDirection(self):
    return self.getForwardVector()

  def getVelocity(self):
    V=numpy.array([self.getChassis().getLinearVelocity()[0],self.getChassis().getLinearVelocity()[1]])
    return numpy.linalg.norm(V)

  def setSensor(self,sensor):
    self.sensor=sensor

  def setAgent(self,agent):
    self.agent=agent

  def controlInput(self,agentInput):
    engineForce = 0.0
    brakeForce = 0.0

    engineForce=agentInput[0]
    self.steering=self.steering+agentInput[1]

    steeringLimit=45

    if self.steering>steeringLimit:
      self.steering=steeringLimit
    if self.steering<-steeringLimit:
      self.steering=-steeringLimit
    
    # Apply steering to front wheels
    self.setSteeringValue(self.steering, 0)
    self.setSteeringValue(self.steering, 1)

    # Apply engine and brake to rear wheels
    self.applyEngineForce(engineForce, 2)
    self.applyEngineForce(engineForce, 3)
    self.setBrake(brakeForce, 2)
    self.setBrake(brakeForce, 3)


