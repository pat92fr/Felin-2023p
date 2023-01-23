import numpy as np

class Configuration:
    def __init__(self):

        #################### COMMANDS ####################
        self.max_x_velocity = 0.9
        self.max_y_velocity = 0.4
        self.max_yaw_rate = 1.8
        self.max_pitch = 30.0 * np.pi / 180.0
        
        #################### MOVEMENT PARAMS ####################
        self.z_time_constant = 0.02
        self.z_speed = 0.04  # maximum speed [m/s]
        self.pitch_deadband = 0.02
        self.pitch_time_constant = 0.25
        self.max_pitch_rate = 0.15
        self.roll_speed = 0.16  # maximum roll rate [rad/s]
        self.yaw_time_constant = 0.3
        self.max_stance_yaw = 0.8
        self.max_stance_yaw_rate = 1.8

        ######################## GEOMETRY ######################
        self.LEG_OX = 0.172  # Leg origin along body X axis
        self.LEG_OY = 0.060  # Leg origin along body Y axis
        self.LEG_OZ = 0.000  # Leg origin along body Z axis
        self.ABDUCTION_OFFSET = 0.06  # distance from abduction axis to leg
        self.LEG_LF = 0.140  # Length of femur (upper leg)
        self.LEG_LT = 0.150  # Length of tibia (lower leg)
        self.FOOT_RADIUS = 0.02

        self.LEG_MIN_LENGTH = 0.060
        self.LEG_MAX_LENGTH = self.LEG_LF + self.LEG_LT - 0.010

        self.LEG_ORIGINS = np.array(
            [
                [ self.LEG_OX, self.LEG_OX, -self.LEG_OX, -self.LEG_OX],
                [-self.LEG_OY, self.LEG_OY, -self.LEG_OY,  self.LEG_OY],
                [ self.LEG_OZ, self.LEG_OZ,  self.LEG_OZ,  self.LEG_OZ],
            ]
        )

        self.ABDUCTION_OFFSETS = np.array(
            [
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
            ]
        )

        #################### CROUCH ####################
        self.CROUCH_X =  0.172
        self.CROUCH_Y =  0.120        
        self.CROUCH_Z = -0.060
        self.CROUCH_X_SHIFT   = 0.020

        #################### STANCE ####################
        self.STANCE_X =  0.172
        self.STANCE_Y =  0.120        
        self.STANCE_Z = -0.180
        self.STANCE_X_SHIFT   = 0.020

        #################### SWING ######################
        self.z_coeffs = None
        self.z_clearance = 0.09 #0.07
        self.alpha = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )
        self.beta = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )

        #################### GAIT #######################
        self.dt = 0.005
        self.num_phases = 4
        self.contact_phases = np.array(
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        )
        self.overlap_time = (
            0.10  # duration of the phase where all four feet are on the ground
        )
        self.swing_time = (
            0.15  # duration of the phase when only two feet are on the ground
        )

    @property
    def default_stance_with_zero_height(self):
        return np.array(
            [
                [
                     self.STANCE_X + self.STANCE_X_SHIFT,
                     self.STANCE_X + self.STANCE_X_SHIFT,
                    -self.STANCE_X + self.STANCE_X_SHIFT,
                    -self.STANCE_X + self.STANCE_X_SHIFT,
                ],
                [
                    -self.STANCE_Y,
                    self.STANCE_Y,
                    -self.STANCE_Y,
                    self.STANCE_Y
                ],
                [0, 0, 0, 0],
            ]
        )

    @property
    def default_stance(self):
        return np.array(
            [
                [
                     self.STANCE_X + self.STANCE_X_SHIFT,
                     self.STANCE_X + self.STANCE_X_SHIFT,
                    -self.STANCE_X + self.STANCE_X_SHIFT,
                    -self.STANCE_X + self.STANCE_X_SHIFT,
                ],
                [
                    -self.STANCE_Y,
                    self.STANCE_Y,
                    -self.STANCE_Y,
                    self.STANCE_Y
                ],
                [
                    self.STANCE_Z,
                    self.STANCE_Z,
                    self.STANCE_Z,
                    self.STANCE_Z
                ],
            ]
        )

    ### default_stance_STANCE_Z = config.default_stance + np.array([[0.0,0.0,config.STANCE_Z_ref],]*4).transpose()

    @property
    def default_crouch_with_zero_height(self):
        return np.array(
            [
                [
                     self.CROUCH_X + self.CROUCH_X_SHIFT,
                     self.CROUCH_X + self.CROUCH_X_SHIFT,
                    -self.CROUCH_X + self.CROUCH_X_SHIFT,
                    -self.CROUCH_X + self.CROUCH_X_SHIFT,
                ],
                [
                    -self.CROUCH_Y,
                    self.CROUCH_Y,
                    -self.CROUCH_Y,
                    self.CROUCH_Y
                ],
                [
                    0,
                    0,
                    0,
                    0
                ],
            ]
        )

    @property
    def default_crouch(self):
        return np.array(
            [
                [
                     self.CROUCH_X + self.CROUCH_X_SHIFT,
                     self.CROUCH_X + self.CROUCH_X_SHIFT,
                    -self.CROUCH_X + self.CROUCH_X_SHIFT,
                    -self.CROUCH_X + self.CROUCH_X_SHIFT,
                ],
                [
                    -self.CROUCH_Y,
                    self.CROUCH_Y,
                    -self.CROUCH_Y,
                    self.CROUCH_Y
                ],
                [
                    self.CROUCH_Z,
                    self.CROUCH_Z,
                    self.CROUCH_Z,
                    self.CROUCH_Z
                ],
            ]
        )

    ################## SWING ###########################

    @property
    def z_clearance(self):
        return self.__z_clearance

    @z_clearance.setter
    def z_clearance(self, z):
        self.__z_clearance = z

    ########################### GAIT ####################
    @property
    def overlap_ticks(self):
        return int(self.overlap_time / self.dt)

    @property
    def swing_ticks(self):
        return int(self.swing_time / self.dt)

    @property
    def stance_ticks(self):
        return 2 * self.overlap_ticks + self.swing_ticks

    @property
    def phase_ticks(self):
        return np.array(
            [self.overlap_ticks, self.swing_ticks, self.overlap_ticks, self.swing_ticks]
        )

    @property
    def phase_length(self):
        return 2 * self.overlap_ticks + 2 * self.swing_ticks

        
