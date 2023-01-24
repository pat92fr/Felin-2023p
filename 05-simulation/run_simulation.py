
import pybullet as p
import pybullet_data

import os
import math
import time
import numpy as np

from src.Controller import Controller
from src.State import State, BehaviorState
from src.Command import Command
from src.JoystickInterface import JoystickInterface
from felin.Config import Configuration
import felin.Kinematics as Kinematics


SCRIPT_PATH = os.path.dirname(os.path.realpath(__file__))
print(SCRIPT_PATH)

class Felin:

  def __init__(self, urdfRootPath=''):
    # load URDF
    self.id = p.loadURDF("%s/felin.urdf" % urdfRootPath, -0.35, 0, .2)

    # get configuration
    self.config = Configuration()

    # leg order FR FL RR RL
    self.legs = [ "FR", "FL", "RR", "RL"]
    # sub order ABDUCTION HIPS KNEE
    self.joints = ["ABD", "HIPS", "KNEE"]

    # alias
    self.nMotors = 12
    self.motorIdList = []
    self.buildJointNameToIdDict()
    self.buildMotorIdList()

    # apply initial pose
    self.resetPose()
    for i in range(100):
      p.stepSimulation()

    # create state
    self.state = State(self.config)

    # create controller and user input handles
    self.controller = Controller(self.config,Kinematics.four_legs_explicit_inverse_kinematics_BRF)

    # create joystick interface
    self.joystick_interface = JoystickInterface(self.config)

    # get initial simulation time
    self.last_loop = time.time()

  def update(self):

    # Read imu data. Orientation will be None if no data was available
    position, orientation = p.getBasePositionAndOrientation(self.id)
    if True: # use IMU (simulated)
      self.state.quat_orientation = np.array([ orientation[3], orientation[0], orientation[1], orientation[2] ]) # xyzw (pybullet) to sxyz (transform3d)
    else:
      self.state.quat_orientation = np.array([1, 0, 0, 0])

    # build command from joystick
    command = self.joystick_interface.get_command(self.state)

    # then run controller and compute new state
    self.controller.run(self.state, command)

    # apply pose from computed state
    self.setPose(self.state.joint_angles,None)

  def buildJointNameToIdDict(self):
    nJoints = p.getNumJoints(self.id)
    print("nJoints:"+str(nJoints))
    self.jointNameToId = {}
    for i in range(nJoints):
      jointInfo = p.getJointInfo(self.id, i)
      self.jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    print("nJoints:"+str(self.jointNameToId))

  def buildMotorIdList(self):
    for leg in self.legs:
      for joint in self.joints:
        self.motorIdList.append(self.jointNameToId['%s-%s' % (leg,joint)])
    print("motorIdList:"+str(self.motorIdList))

  def setMotorAngleById(self, joint_index, joint_position, joint_speed = None): 
    if joint_speed is None:
      p.setJointMotorControl2(bodyIndex=self.id,
                              jointIndex=joint_index,
                              controlMode=p.POSITION_CONTROL,
                              targetPosition=joint_position,
                              positionGain=0.15,
                              #velocityGain=xxxx,
                              force=self.config.MAX_JOINT_TORQUE)
    else:
      p.setJointMotorControl2(bodyIndex=self.id,
                              jointIndex=joint_index,
                              controlMode=p.POSITION_CONTROL,
                              targetPosition=joint_position,
                              targetVelocity=joint_speed,
                              positionGain=0.15,
                              #velocityGain=xxxx,
                              force=self.config.MAX_JOINT_TORQUE)


  def setMotorAngleByName(self, joint_name, joint_position, joint_speed = None ):
    self.setMotorAngleById(self.jointNameToId[joint_name], joint_position, joint_speed)

  def setPose(self, joint_positions, joint_speeds):
    for leg_index in range(4):
      for joint_index in range(3):
        joint_name = '%s-%s' % (self.legs[leg_index],self.joints[joint_index])
        self.setMotorAngleById(self.jointNameToId[joint_name], joint_positions[joint_index,leg_index])

  def resetPose(self):
    # stand-by pose
    feet_positions_BRF = self.config.default_crouch
    print("feet_positions_BRF:\n"+str(feet_positions_BRF))
    joint_positions = Kinematics.four_legs_explicit_inverse_kinematics_BRF(feet_positions_BRF,self.config)
    print("joint_positions:\n\n"+str(np.round(np.degrees(joint_positions),1))+"\n")

    # reset joint state
    for leg_index in range(4):
      for joint_index in range(3):
        joint_name = '%s-%s' % (self.legs[leg_index],self.joints[joint_index])
        p.resetJointState(self.id, self.jointNameToId[joint_name], joint_positions[joint_index,leg_index])
        print("set joint position for "+joint_name+" = "+str(joint_positions[joint_index,leg_index]))

    # set pose with joint position and zero joint speed
    self.setPose(joint_positions,np.zeros((3,4)))

  def getBasePosition(self):
    position, orientation = p.getBasePositionAndOrientation(self.id)
    return position

  def getBaseOrientation(self):
    position, orientation = p.getBasePositionAndOrientation(self.id)
    return orientation

  def getMotorAngles(self):
    motorAngles = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.id, self.motorIdList[i])
      motorAngles.append(jointState[0])
    #motorAngles = np.multiply(motorAngles, self.motorDir)
    return motorAngles

  def getMotorVelocities(self):
    motorVelocities = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.id, self.motorIdList[i])
      motorVelocities.append(jointState[1])
    #motorVelocities = np.multiply(motorVelocities, self.motorDir)
    return motorVelocities

  def getMotorTorques(self):
    motorTorques = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.id, self.motorIdList[i])
      motorTorques.append(jointState[3])
    #motorTorques = np.multiply(motorTorques, self.motorDir)
    return motorTorques


class simulator:
  '''
    Init pyBullet
    Init custom GUI
    Load 3D environnement (Toulouse Robot Race)
    Load Robot
    Init Camera
    Init simulation time
  '''

  # doc: https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstart_guide/PyBulletQuickstartGuide.md.html

  def __init__(self):

    # Init pyBullet
    physicsClient = p.connect(p.GUI) #, options="--opengl2")#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-9.81)

    # Init custom GUI
    self.cam_sel = p.addUserDebugParameter("3RD PERSON CAMERA", 0, 1, 0)
    self.cam_dist = p.addUserDebugParameter("CAMERA DISTANCE", 0.5, 4.0, 1.0)
    self.cam_onboard = p.addUserDebugParameter("1RST CAMERA", 0, 1, 0)

    # Load 3D environnement (Toulouse Robot Race)
    
    # Load ground plane
    planeId = p.loadURDF("plane.urdf")

    # Create Ark
    arkFile = os.path.join(SCRIPT_PATH, 'models/ark.obj')
    arkVisualId = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=arkFile, rgbaColor=[1,1,1,1], specularColor=[0.4, 0.4,0])
    arkCollisionId = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=arkFile, collisionFramePosition=[0, 0, 0])
    arkId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=arkVisualId, basePosition=[10, 0, 0], baseOrientation=p.getQuaternionFromEuler([3.14, 0, 3.14/2]))

    # Create Walls
    wallVisualId = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[7.5, 0.020, 0.075], rgbaColor=[0.9, 0.9, 0.9, 1], specularColor=[0.8, 0.8, 0.8])
    wallCollisionId = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[7.5, 0.020, 0.075])
    wallLeftId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wallCollisionId, baseVisualShapeIndex=wallVisualId, basePosition=[7, 0.750, 0.075], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))
    ## Right wall (15000, 40, 150)mm d=1500
    wallRightId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wallCollisionId, baseVisualShapeIndex=wallVisualId, basePosition=[7, -0.750, 0.075], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))

    # Create lines
    ## Middle line
    mLineVisualId = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[7.5, 0.025, 0.001], rgbaColor=[0, 0, 0, 1], specularColor=[0.4, 0.4, 0])
    mLineId = p.createMultiBody(baseMass=0, baseVisualShapeIndex=mLineVisualId, basePosition=[7, 0, 0.001], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))

    ## Start and End line
    ### Start line
    sLineVisualId = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[0.025, 0.750, 0.001], rgbaColor=[0, 1, 0, 1], specularColor=[0.4, 0.4, 0])
    sLineId = p.createMultiBody(baseMass=0, baseVisualShapeIndex=sLineVisualId, basePosition=[0, 0, 0.001], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))

    ### End line
    eLineVisualId = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[0.025, 0.750, 0.001], rgbaColor=[1, 0, 0, 1], specularColor=[0.4, 0.4, 0])
    eLineId = p.createMultiBody(baseMass=0, baseVisualShapeIndex=eLineVisualId, basePosition=[10, 0, 0.001], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))

    # Load Robot
    self.quadruped = Felin("felin/")

    # Init Camera
    quadruped_position, quadruped_attitude = p.getBasePositionAndOrientation(self.quadruped.id) 
    self.quadruped_position_filtered = (quadruped_position[0],quadruped_position[1],0)
    p.resetDebugVisualizerCamera(
      cameraDistance=1.0,
      cameraYaw=40,
      cameraPitch=-40,
      cameraTargetPosition=self.quadruped_position_filtered
    )

    # Init simulation time
    p.setRealTimeSimulation(False)
    p.setTimeStep(self.quadruped.config.dt) # synchronise controller refresh rate with simulation step


  '''
    Run simulation
    Init Race chrono (Toulouse Robot Race)
    Init Log file
    Use robot controller
    Step simulation
    Log simulation data
    Update Third-Person View Camera
    Render on-board camera (option)
  '''
  def run(self):

    # Init Race chrono (Toulouse Robot Race)
    start_time = 0
    finish_time = 0
    current_time_s = 0
    started = False
    finished = False
    chrono_displayed = False

    # Init log
    log_body = open("dataset_pybullet_body.txt", "w")
    log_body.write("x,y,z,p,r,y\n")

    # game loop
    while True:

      # force stand-by pose (debug)
      if False:
        feet_positions_BRF = self.quadruped.config.default_stance_with_z
        joint_positions = Kinematics.four_legs_explicit_inverse_kinematics_BRF(feet_positions_BRF,self.quadruped.config)
        joint_speeds = None
        self.quadruped.setPose(joint_positions,joint_speeds)
      # use controller pose
      else:
        self.quadruped.update()

      # run simulation step
      p.stepSimulation()

      # Update camera
      quadruped_position, quadruped_attitude = p.getBasePositionAndOrientation(self.quadruped.id) 
      alpha = 0.1
      self.quadruped_position_filtered = (
        (1.0-alpha)*self.quadruped_position_filtered[0]+alpha*quadruped_position[0],
        (1.0-alpha)*self.quadruped_position_filtered[1]+alpha*quadruped_position[1],
        0
      )

      # Update Camera
      if p.readUserDebugParameter(self.cam_sel) <0.2: # 3/4 view
        p.resetDebugVisualizerCamera(
          cameraDistance=p.readUserDebugParameter(self.cam_dist),
          cameraYaw=-40,
          cameraPitch=-40,
          cameraTargetPosition=self.quadruped_position_filtered
        )

      elif p.readUserDebugParameter(self.cam_sel) <0.4: # side view
        p.resetDebugVisualizerCamera(
          cameraDistance=p.readUserDebugParameter(self.cam_dist),
          cameraYaw=0,
          cameraPitch=-5,
          cameraTargetPosition=self.quadruped_position_filtered
        )

      elif p.readUserDebugParameter(self.cam_sel) <0.6: # top view
        p.resetDebugVisualizerCamera(
          cameraDistance=p.readUserDebugParameter(self.cam_dist),
          cameraYaw=-90,
          cameraPitch=-89,
          cameraTargetPosition=self.quadruped_position_filtered
        )
 
      elif p.readUserDebugParameter(self.cam_sel) <0.8: # side view
        p.resetDebugVisualizerCamera(
          cameraDistance=p.readUserDebugParameter(self.cam_dist),
          cameraYaw=-180,
          cameraPitch=-10,
          cameraTargetPosition=self.quadruped_position_filtered
        )

      elif p.readUserDebugParameter(self.cam_sel) <0.95: # rear view
        p.resetDebugVisualizerCamera(
          cameraDistance=p.readUserDebugParameter(self.cam_dist),
          cameraYaw=-90,
          cameraPitch=-10,
          cameraTargetPosition=self.quadruped_position_filtered
        )        

      else: # front view
        p.resetDebugVisualizerCamera(
          cameraDistance=p.readUserDebugParameter(self.cam_dist),
          cameraYaw=90,
          cameraPitch=-10,
          cameraTargetPosition=self.quadruped_position_filtered
        )  

      # race chrono update
      current_time_s += self.quadruped.config.dt
      if not started and not finished:
        if self.quadruped_position_filtered[0] > 0:
          started = True
          start_time = current_time_s
          finished = False
      else:
        if self.quadruped_position_filtered[0] > 10.0 and not finished:
          started = True
          finished = True
          finish_time = current_time_s
          chronotime = finish_time-start_time
      if finished and current_time_s > finish_time+0.3:
        if not chrono_displayed:
          chrono_displayed = True
          p.addUserDebugText(
           "Chrono: "+str(round(chronotime,1))+"s",
           (self.quadruped_position_filtered[0],self.quadruped_position_filtered[1],self.quadruped_position_filtered[2]+0.3),
           #replaceItemUniqueId=chrono_text_id,
           textSize = 3,
           textColorRGB= [0,0,0]
           )

      # Views
      pitch,roll,yaw = p.getEulerFromQuaternion(quadruped_attitude)
      if p.readUserDebugParameter(self.cam_onboard)>0.5:
        viewMatrix = p.computeViewMatrix(
          cameraEyePosition=[quadruped_position[0]+0.3*math.cos(yaw), quadruped_position[1]+0.3*math.sin(yaw), quadruped_position[2]+0.1],
          cameraTargetPosition=[quadruped_position[0]+0.5*math.cos(yaw), quadruped_position[1]+0.5*math.sin(yaw), quadruped_position[2]+0.1],
          cameraUpVector=[0, 0, 1]
        )
        projectionMatrix = p.computeProjectionMatrixFOV(
          fov=45.0,
          aspect=1.0,
          nearVal=0.1,
          farVal=13.1
        )        
        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
          width=224, 
          height=224,
          viewMatrix=viewMatrix,
          projectionMatrix=projectionMatrix
        )

      # log into file  
      log_body.write(
         "{:.3f}".format(self.quadruped_position_filtered[0])+","+
         "{:.3f}".format(self.quadruped_position_filtered[1])+","+
         "{:.3f}".format(self.quadruped_position_filtered[2])+","+
         "{:.3f}".format(pitch)+","+
         "{:.3f}".format(roll)+","+
         "{:.3f}".format(yaw)+"\n"
      )

    p.disconnect()

if __name__ == "__main__":
  s = simulator()
  s.run()