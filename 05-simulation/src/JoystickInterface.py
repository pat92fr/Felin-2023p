import UDPComms
import numpy as np
import time
from src.State import BehaviorState, State
from src.Command import Command
from src.Utilities import deadband, clipped_first_order_filter


class JoystickInterface:
    def __init__(
        self, config, udp_port=8830, udp_publisher_port = 8840,
    ):
        self.config = config
        self.previous_state = BehaviorState.CROUCH
        self.previous_activate_toggle = 0
        self.previous_trot_toggle = 0
        self.previous_hop_toggle = 0

        self.message_rate = 50
        self.udp_handle = UDPComms.Subscriber(udp_port, timeout=0.3)
        self.udp_publisher = UDPComms.Publisher(udp_publisher_port)


    def get_command(self, state, do_print=False):
        try:
            msg = self.udp_handle.get()
            command = Command()
            
            ####### Handle discrete commands ########
            activate_toggle = msg["L1"]
            command.activate_event = (activate_toggle == 1 and self.previous_activate_toggle == 0)

            # Check if requesting a state transition to trotting, or from trotting to resting
            trot_toggle = msg["R1"]
            command.trot_event = (trot_toggle == 1 and self.previous_trot_toggle == 0)

            # Check if requesting a state transition to hopping, from trotting or resting
            hop_toggle = msg["x"]
            command.hop_event = (hop_toggle == 1 and self.previous_hop_toggle == 0)            
            
            # Update previous values for toggles and state
            self.previous_activate_toggle = activate_toggle
            self.previous_trot_toggle = trot_toggle
            self.previous_hop_toggle = hop_toggle

            ####### Handle continuous commands ########
            message_rate = msg["message_rate"]
            message_dt = 1.0 / message_rate

            x_vel = msg["ly"] * self.config.max_x_velocity
            y_vel = msg["lx"] * -self.config.max_y_velocity
            command.horizontal_velocity = np.array([x_vel, y_vel])
            command.yaw_rate = msg["rx"] * -self.config.max_yaw_rate

            pitch = msg["ry"] * self.config.max_pitch
            deadbanded_pitch = deadband(
                pitch, self.config.pitch_deadband
            )
            pitch_rate = clipped_first_order_filter(
                state.pitch,
                deadbanded_pitch,
                self.config.max_pitch_rate,
                self.config.pitch_time_constant,
            )
            command.pitch = state.pitch + message_dt * pitch_rate

            height_movement = msg["dpady"]
            #command.height = state.height - message_dt * self.config.z_speed * height_movement
            command.height = self.config.STANCE_Z
            
            roll_movement = - msg["dpadx"]
            command.roll = state.roll + message_dt * self.config.roll_speed * roll_movement

            return command

        except UDPComms.timeout:
            if do_print:
                print("UDP Timed out")
            return Command()


    def set_color(self, color):
        joystick_msg = {"ps4_color": color}
        self.udp_publisher.send(joystick_msg)