
#from pandac.PandaModules import loadPrcFileData
#loadPrcFileData('', 'load-display tinydisplay')

import sys
import direct.directbase.DirectStart
import numpy as np
import math

from direct.showbase.DirectObject import DirectObject
from direct.showbase.InputStateGlobal import inputState

from panda3d.core import AmbientLight
from panda3d.core import DirectionalLight
from panda3d.core import Vec3
from panda3d.core import Vec4
from panda3d.core import Point3
from panda3d.core import TransformState
from panda3d.core import BitMask32

from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletPlaneShape
from panda3d.bullet import BulletBoxShape
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletDebugNode
from panda3d.bullet import BulletVehicle
from panda3d.bullet import ZUp
from vehicle import *
from road import *
from sensor import *
from agent import *


from panda3d.core import *
import sys
import os

import direct.directbase.DirectStart
from direct.interval.IntervalGlobal import *
from direct.gui.DirectGui import OnscreenText
from direct.showbase.DirectObject import DirectObject
from direct.actor import Actor
from random import *



class Game(DirectObject):

  def __init__(self):
    base.setBackgroundColor(0.1, 0.1, 0.8, 1)
    base.setFrameRateMeter(True)

    # road geometry generation
    #road=trail(100,50,40,0.2) # generating a circular lane track
    self.precision=0.5 # length of a piece of centerLine
    road=basicFreeWay(200,50,1,self.precision,2) # generating a piece of freeway
    #road=straightCenter(np.array([0,0]),math.pi/2,300,2)
    self.segLine=road.getLine() # the centerLine
    self.road=roadGenerator(np.array([0,-1]),4,2,road.getFollowing(),self.segLine,-1) # generate road polygon
    node=self.road.getNode()

    # road texture
    floorTex = loader.loadTexture('maps/street3.jpg')
    floor = render.attachNewNode(node)
    floor.setTexture(floorTex)
    floor.flattenStrong()
    
    # grass background generation
    floorTex1 = loader.loadTexture('maps/envir-ground.jpg')
    cm1 = CardMaker('')
    cm1.setFrame(-2, 2, -2, 2)
    floor1 = render.attachNewNode(PandaNode("floor1"))
    for y in range(100):
        for x in range(30):
            nn1 = floor1.attachNewNode(cm1.generate())
            nn1.setP(-90)
            nn1.setPos((x - 20) * 4, (y - 20) * 4, -1.1)
    floor1.setTexture(floorTex1)
    floor1.flattenStrong()

    # initial camera
    base.cam.setPos(0, -20, 4)
    base.cam.lookAt(0, 0, 0)

    # Light
    alight = AmbientLight('ambientLight')
    alight.setColor(Vec4(0.5, 0.5, 0.5, 1))
    alightNP = render.attachNewNode(alight)

    dlight = DirectionalLight('directionalLight')
    dlight.setDirection(Vec3(1, 1, -1))
    dlight.setColor(Vec4(0.7, 0.7, 0.7, 1))
    dlightNP = render.attachNewNode(dlight)

    render.clearLight()
    render.setLight(alightNP)
    render.setLight(dlightNP)

    # Input setup
    self.accept('escape', self.doExit)
    self.accept('r', self.doReset)
    self.accept('f1', self.toggleWireframe)
    self.accept('f2', self.toggleTexture)
    self.accept('f3', self.toggleDebug)
    self.accept('f5', self.doScreenshot)

    inputState.watchWithModifiers('forward', 'w')
    inputState.watchWithModifiers('reverse', 's')
    inputState.watchWithModifiers('turnLeft', 'a')
    inputState.watchWithModifiers('turnRight', 'd')
    inputState.watchWithModifiers('brake1', 'x')

    inputState.watchWithModifiers('For', 'i')
    inputState.watchWithModifiers('Back', 'k')
    inputState.watchWithModifiers('Lef', 'j')
    inputState.watchWithModifiers('Righ', 'l')
    inputState.watchWithModifiers('brake2', 'space')
    

    # Task manager
    taskMgr.add(self.update, 'updateWorld')

    # Physics
    self.setup()

  # _____HANDLER_____

  def doExit(self):
    self.cleanup()
    sys.exit(1)

  def doReset(self):
    self.cleanup()
    self.setup()

  def toggleWireframe(self):
    base.toggleWireframe()

  def toggleTexture(self):
    base.toggleTexture()

  def toggleDebug(self):
    if self.debugNP.isHidden():
      self.debugNP.show()
    else:
      self.debugNP.hide()

  def doScreenshot(self):
    base.screenshot('Bullet')


  # control camera
  def updateCamera(self):
    #current state of vehicle
    direction=self.vehicles[0].getDirection()
    position=self.vehicles[0].getPos()
    #camera
    base.cam.setPos(position[0]-15*direction[0], position[1]-15*direction[1], 4)
    base.cam.lookAt(position)

  # simulation update per step
  def update(self, task):
    dt = globalClock.getDt()

    #print self.vehicles[0].getVelocity()

    self.vehicles[0].controlInput(self.vehicles[0].agent.doControl())  # vehicle control update
    #self.vehicles[0].processInput(dt,'forward','reverse','turnLeft','turnRight','brake1')
    #self.vehicles[1].processInput(dt,'For','Back','Lef','Righ','brake2')
    self.world.doPhysics(dt, 10, 0.008)

    self.updateCamera()

    # do not remove this
    cordPos=self.vehicles[0].sensor.getCordPos()
    
    return task.cont

  # exit
  def cleanup(self):
    self.world = None
    self.worldNP.removeNode()

  # physical world setup
  def setup(self):
    self.worldNP = render.attachNewNode('World')

    # World
    self.debugNP = self.worldNP.attachNewNode(BulletDebugNode('Debug'))
    self.debugNP.show()

    self.world = BulletWorld()
    self.world.setGravity(Vec3(0, 0, -9.81))
    self.world.setDebugNode(self.debugNP.node())

    # Plane
    shape = BulletPlaneShape(Vec3(0, 0, 1), 0)

    np = self.worldNP.attachNewNode(BulletRigidBodyNode('Ground'))
    np.node().addShape(shape)
    np.setPos(0, 0, -1)
    np.setCollideMask(BitMask32.allOn())

    self.world.attachRigidBody(np.node())

    # initial vehicles
    self.vehicles=[]
    #self.vehicles.append(basicVehicle(self,[13,2,0.5],18)) # [10,0.1,0.5] is vehicle start position
    self.vehicles.append(basicVehicle(self,[10,0.1,0.5],18)) # [10,0.1,0.5] is vehicle start position
    sensor=basicSensor(self)  # initial sensor
    sensor.setVehicle(self.vehicles[0])
    self.vehicles[0].setSensor(sensor)
    self.vehicles[0].sensor.align()
    agent=basicAgent(50,10.8,15)   # initial agent
    agent.setVehicle(self.vehicles[0])
    self.vehicles[0].setAgent(agent)
    #self.vehicles.append(basicVehicle(self,[10,-2,0.5],20))
    

game = Game()
base.run()
