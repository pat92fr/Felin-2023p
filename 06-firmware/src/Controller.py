from src.Gaits import GaitController
from src.StanceController import StanceController
from src.SwingLegController import SwingController
from src.Utilities import clipped_first_order_filter
from src.State import BehaviorState, State

import numpy as np
from transforms3d.euler import euler2mat, quat2euler
from transforms3d.quaternions import qconjugate, quat2axangle
from transforms3d.axangles import axangle2mat


class Controller:
    """Controller and planner object
    """

    def __init__(
        self,
        config,
        inverse_kinematics,
    ):
        self.config = config

        self.smoothed_yaw = 0.0  # for REST mode only
        self.inverse_kinematics = inverse_kinematics

        self.contact_modes = np.zeros(4)
        self.gait_controller = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)


    def step_gait(self, state, command):
        """Calculate the desired foot locations for the next timestep

        Returns
        -------
        Numpy array (3, 4)
            Matrix of new foot locations.
        """
        contact_modes = self.gait_controller.contacts(state.ticks)
        new_foot_locations = np.zeros((3, 4))
        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            foot_location = state.foot_locations[:, leg_index]
            if contact_mode == 1:
                new_location = self.stance_controller.next_foot_location(leg_index, state, command)
            else:
                swing_proportion = (
                    self.gait_controller.subphase_ticks(state.ticks) / self.config.swing_ticks
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion,
                    leg_index,
                    state,
                    command
                )
            new_foot_locations[:, leg_index] = new_location
        return new_foot_locations, contact_modes


    def run(self, state, command):
        """Steps the controller forward one timestep

        Parameters
        ----------
        controller : Controller
            Robot controller object.
        """

        ########## Update operating state based on command ######

        if state.behavior_state == BehaviorState.CROUCH:

            # force body height
            command.height = self.config.CROUCH_Z

            # set state foot position to CROUCH
            state.foot_locations = self.config.default_crouch_with_zero_height + np.array([0, 0, state.height])[:, np.newaxis]
            state.joint_angles = self.inverse_kinematics(state.foot_locations, self.config)            

            # transition to REST
            if command.activate_event:
                state.behavior_state = BehaviorState.STANDING_UP

        elif state.behavior_state == BehaviorState.STANDING_UP:

            # change position from crouch to rest with speed
            speed = np.array([0.025,0.005,0.025])
            for leg_index in range(4):
                state.foot_locations[:,leg_index] = reach(state.foot_locations[:,leg_index],self.config.default_stance[:,leg_index],speed,self.config.dt)

            state.joint_angles = self.inverse_kinematics(state.foot_locations, self.config)            

            if np.linalg.norm(state.foot_locations-self.config.default_stance) < 0.001:
                state.behavior_state = BehaviorState.REST  
                state.height = self.config.STANCE_Z

        elif state.behavior_state == BehaviorState.SITTING_DOWN:

            # change position from crouch to rest with speed
            speed = np.array([0.005,0.005,0.025])
            for leg_index in range(4):
                state.foot_locations[:,leg_index] = reach(state.foot_locations[:,leg_index],self.config.default_crouch[:,leg_index],speed,self.config.dt)

            state.joint_angles = self.inverse_kinematics(state.foot_locations, self.config)            

            if np.linalg.norm(state.foot_locations-self.config.default_crouch) < 0.001:
                state.behavior_state = BehaviorState.CROUCH
                state.height = self.config.CROUCH_Z


        elif state.behavior_state == BehaviorState.TROT:
            state.foot_locations, contact_modes = self.step_gait(
                state,
                command,
            )

            # Apply the desired body rotation
            rotated_foot_locations = (
                euler2mat(
                    command.roll, command.pitch, 0.0
                )
                @ state.foot_locations
            )

            # Construct foot rotation matrix to compensate for body tilt
            (roll, pitch, yaw) = quat2euler(state.quat_orientation)
            correction_factor = 0.8
            max_tilt = 0.4
            roll_compensation = correction_factor * np.clip(roll, -max_tilt, max_tilt)
            pitch_compensation = correction_factor * np.clip(pitch, -max_tilt, max_tilt)
            rmat = euler2mat(roll_compensation, pitch_compensation, 0)

            rotated_foot_locations = rmat.T @ rotated_foot_locations

            # Pat92fr
            # Pat92fr
            rotated_foot_locations = rotated_foot_locations + np.array([-command.horizontal_velocity[0]*0.020,0.0,abs(command.horizontal_velocity[0])*0.020])[:, np.newaxis]
            #print(command.horizontal_velocity[0]*0.020)
            # Pat92fr
            # Pat92fr

            state.joint_angles = self.inverse_kinematics(
                rotated_foot_locations, self.config
            )

            # transition to CROUCH
            if command.activate_event:
                state.behavior_state = BehaviorState.SITTING_DOWN

            # transition to TROT
            if command.trot_event:
                state.behavior_state = BehaviorState.REST


        elif state.behavior_state == BehaviorState.HOP:
            state.foot_locations = (
                self.config.default_stance_with_zero_height
                + np.array([0, 0, -0.09])[:, np.newaxis]
            )
            state.joint_angles = self.inverse_kinematics(
                state.foot_locations, self.config
            )

            # transition to HOP
            if command.hop_event:
                state.behavior_state = BehaviorState.FINISHHOP

        elif state.behavior_state == BehaviorState.FINISHHOP:
            state.foot_locations = (
                self.config.default_stance_with_zero_height
                + np.array([0, 0, -0.26])[:, np.newaxis]
            )
            state.joint_angles = self.inverse_kinematics(
                state.foot_locations, self.config
            )

            # transition to HOP
            if command.hop_event:
                state.behavior_state = BehaviorState.REST

        elif state.behavior_state == BehaviorState.REST:
            yaw_proportion = command.yaw_rate / self.config.max_yaw_rate
            self.smoothed_yaw += (
                self.config.dt
                * clipped_first_order_filter(
                    self.smoothed_yaw,
                    yaw_proportion * -self.config.max_stance_yaw,
                    self.config.max_stance_yaw_rate,
                    self.config.yaw_time_constant,
                )
            )
            # Set the foot locations to the default stance plus the standard height
            state.foot_locations = self.config.default_stance_with_zero_height + np.array([0, 0, state.height])[:, np.newaxis]
            # Apply the desired body rotation
            rotated_foot_locations = (
                euler2mat(
                    command.roll,
                    command.pitch,
                    self.smoothed_yaw,
                )
                @ state.foot_locations
            )
            state.joint_angles = self.inverse_kinematics(
                rotated_foot_locations, self.config
            )

            # transition to CROUCH
            if command.activate_event:
                state.behavior_state = BehaviorState.SITTING_DOWN

            # transition to TROT
            if command.trot_event:
                state.behavior_state = BehaviorState.TROT

            # transition to HOP
            if command.hop_event:
                state.behavior_state = BehaviorState.HOP

        state.ticks += 1
        state.pitch = command.pitch
        state.roll = command.roll

        # update body height according command and maximum body Z speed
        # if state.height > command.height:
        #     state.height = max( command.height, state.height-self.config.z_speed*self.config.dt)
        # if state.height < command.height:
        #     state.height = min( command.height, state.height+self.config.z_speed*self.config.dt)


## from one pose to another one, with speed control
def reach(pose,target,speed,dt):
    delta = target-pose
    delta_abs = np.absolute(delta)
    max_delta_abs = speed*dt
    real_delta = np.minimum(delta_abs,max_delta_abs)
    return pose + np.sign(delta)*real_delta
    
