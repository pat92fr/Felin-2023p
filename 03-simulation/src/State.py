import numpy as np
from enum import Enum

class State:
    def __init__(self,config):
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0.0
        self.height = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.activation = 0
        self.behavior_state = BehaviorState.REST

        self.ticks = 0
        
        # setpoint
        self.foot_locations = np.zeros((3, 4))
        self.joint_angles = np.zeros((3, 4))
        self.joint_torques = np.zeros((3, 4))

        # feedback
        self.joint_present_angles = np.zeros((3, 4))
        self.joint_present_torques = np.zeros((3, 4))

        self.behavior_state = BehaviorState.REST


class BehaviorState(Enum):
    DEACTIVATED = -1
    REST = 0
    TROT = 1
    HOP = 2
    FINISHHOP = 3