import numpy as np

class Configuration:
    def __init__(self):

        #################### COMMANDS ####################
        self.max_x_velocity = 0.4
        self.max_y_velocity = 0.3
        self.max_yaw_rate = 2.0
        self.max_pitch = 30.0 * np.pi / 180.0
        
        #################### MOVEMENT PARAMS ####################
        self.z_time_constant = 0.02
        self.z_speed = 0.03  # maximum speed [m/s]
        self.pitch_deadband = 0.02
        self.pitch_time_constant = 0.25
        self.max_pitch_rate = 0.15
        self.roll_speed = 0.16  # maximum roll rate [rad/s]
        self.yaw_time_constant = 0.3
        self.max_stance_yaw = 1.2
        self.max_stance_yaw_rate = 2.0

        ######################## GEOMETRY ######################
        # self.LEG_FB = 0.172  # front-back distance from center line to leg axis
        # self.LEG_LR = 0.060  # left-right distance from center line to leg plane
        self.LEG_OX = 0.172  # Leg origin along body X axis
        self.LEG_OY = 0.060  # Leg origin along body Y axis
        self.LEG_OZ = 0.000  # Leg origin along body Z axis
        self.ABDUCTION_OFFSET = 0.06  # distance from abduction axis to leg
        self.LEG_LF = 0.140  # Length of femur (upper leg)
        self.LEG_LT = 0.150  # Length of tibia (lower leg)
        self.FOOT_RADIUS = 0.02

        self.LEG_MIN_LENGTH = 0.060
        self.LEG_MAX_LENGTH = self.LEG_LF + self.LEG_LT - 0.010

        # self.HIP_L = 0.0394
        # self.HIP_W = 0.0744
        # self.HIP_T = 0.0214
        # self.HIP_OFFSET = 0.0132

        # self.L = 0.276
        # self.W = 0.100
        # self.T = 0.050

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
        #################### STANCE ####################
        # self.delta_x = 0.172
        # self.delta_y = 0.120
        self.DEFAULT_X =  0.172
        self.DEFAULT_Y =  0.140        
        self.DEFAULT_Z = -0.180
        self.X_SHIFT   = -0.020

        #################### SWING ######################
        self.z_coeffs = None
        self.z_clearance = 0.07
        self.alpha = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )
        self.beta = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )

        #################### GAIT #######################
        self.dt = 0.01
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
    def default_stance(self):
        return np.array(
            [
                [
                     self.DEFAULT_X + self.X_SHIFT,
                     self.DEFAULT_X + self.X_SHIFT,
                    -self.DEFAULT_X + self.X_SHIFT,
                    -self.DEFAULT_X + self.X_SHIFT,
                ],
                [
                    -self.DEFAULT_Y,
                    self.DEFAULT_Y,
                    -self.DEFAULT_Y,
                    self.DEFAULT_Y
                ],
                [0, 0, 0, 0],
            ]
        )

    @property
    def default_stance_with_z(self):
        return np.array(
            [
                [
                     self.DEFAULT_X + self.X_SHIFT,
                     self.DEFAULT_X + self.X_SHIFT,
                    -self.DEFAULT_X + self.X_SHIFT,
                    -self.DEFAULT_X + self.X_SHIFT,
                ],
                [
                    -self.DEFAULT_Y,
                    self.DEFAULT_Y,
                    -self.DEFAULT_Y,
                    self.DEFAULT_Y
                ],
                [
                    self.DEFAULT_Z,
                    self.DEFAULT_Z,
                    self.DEFAULT_Z,
                    self.DEFAULT_Z
                ],
            ]
        )

    ### default_stance_default_z = config.default_stance + np.array([[0.0,0.0,config.default_z_ref],]*4).transpose()


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

        
