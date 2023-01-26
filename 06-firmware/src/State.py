import numpy as np
from enum import Enum

class State:
    def __init__(self,config):

        self.behavior_state = BehaviorState.CROUCH
        self.ticks = 0

        # input data
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0.0
        self.height = config.CROUCH_Z
        self.pitch = 0.0
        self.roll = 0.0

        # setpoint
        self.foot_locations = config.default_crouch
        self.joint_angles = np.zeros((3, 4))
        self.joint_torques = np.zeros((3, 4))

        # feedback
        self.joint_present_angles = np.zeros((3, 4))
        self.joint_present_torques = np.zeros((3, 4))


class BehaviorState(Enum):
    DEACTIVATED = -1
    CROUCH = 0
    REST = 1
    TROT = 2
    HOP = 3
    FINISHHOP = 4