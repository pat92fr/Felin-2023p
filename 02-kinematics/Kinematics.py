import math
import numpy as np
from transforms3d.euler import euler2mat


##DEBUG #######################################################################

# TRACE macro
DEBUG = True
DEBUG = False

def log(s):
    if DEBUG:
        print(s)

##DEBUG #######################################################################

# Constant Rotation from BRF to LRF reference frame
R_BRF_to_LRF = np.array([
        [0,0,1],
        [0,1,0],
        [-1,0,0]
    ])


def leg_forward_kinematics_LRF(joint_position, leg_index, config):
    """Find the foot position in the Leg Reference Frame

    LFR : centered HIPS axis, X upward, Y leftward, Z backward for all leg

    Parameters
    ----------
    joint_position : numpy array (3)
        Array of the joint angles.
        [0] hips abduction revolute joint in RADIANS
        [1] hips flexion/extension revolute joint in RADIANS
        [2] knee flexion/extension revolute joint in RADIANS
    leg_index :
        Index of the current leg
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    config : [type]
        [description]

    Returns
    -------
    numpy array (3)
        Array of corresponding foot positions in Leg Reference Frame
        [X,Y,Z] in LRF
    """
    c1 = math.cos(joint_position[0])
    s1 = math.sin(joint_position[0])
    c2 = -math.sin(joint_position[1])
    s2 = math.cos(joint_position[1])
    c23 = math.sin(joint_position[1]+joint_position[2])
    s23 = -math.cos(joint_position[1]+joint_position[2])
    x = c1*c23*config.LEG_LT + c1*c2*config.LEG_LF - s1*config.ABDUCTION_OFFSETS[leg_index]
    y = s1*c23*config.LEG_LT + s1*c2*config.LEG_LF + c1*config.ABDUCTION_OFFSETS[leg_index]
    z =   -s23*config.LEG_LT -    s2*config.LEG_LF
    return np.array([x,y,z])


def four_legs_forward_kinematics_LRF(joint_position, config):
    """Find the position of the four feet in their respective Leg Reference Frame

    LFR : centered HIPS axis, X upward, Y leftward, Z backward for all leg

    Parameters
    ----------
    joint_position : numpy array (3x4) 
        Array of the joint angles of four legs
        [0,i] hips abduction revolute joint in RADIANS
        [1,i] hips flexion/extension revolute joint in RADIANS
        [2,i] knee flexion/extension revolute joint in RADIANS
    where i is the leg_index :
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    config : [type]
        [description]

    Returns
    -------
    numpy array (3x4)
        Array of corresponding foot positions in Leg Reference Frame
        [X,Y,Z] in LRF
    """    
    feet_position_LRF = np.zeros((3,4))
    # for each leg
    for i in range(4):
        feet_position_LRF[:,i] = leg_forward_kinematics_LRF(joint_position[:,i],i,config)
    return feet_position_LRF


def four_legs_forward_kinematics_BRF(joint_position, config):
    """Find the position of the four feet in the Body Reference Frame

    BRF : centered body, X forward, Y leftward, Z upward for all leg

    Parameters
    ----------
    joint_position : numpy array (3x4) 
        Array of the joint angles of four legs
        [0,i] hips abduction revolute joint in RADIANS
        [1,i] hips flexion/extension revolute joint in RADIANS
        [2,i] knee flexion/extension revolute joint in RADIANS
    where i is the leg_index :
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    config : [type]
        [description]

    Returns
    -------
    numpy array (3x4)
        Array of corresponding foot positions in Body Reference Frame
        [X,Y,Z] in LRF
    """    
    feet_position_BRF = np.zeros((3,4))
    # for each leg
    for i in range(4):
        feet_position_LRF = leg_forward_kinematics_LRF(joint_position[:,i],i,config)
        feet_position_BRF[:,i] = R_BRF_to_LRF.transpose().dot(feet_position_LRF)+config.LEG_ORIGINS[:,i]
    return feet_position_BRF

def jacobian(joint_position,leg_index,config):
    """Compute Jacobian matrixin Leg Reference Frame

    LFR : centered HIPS axis, X upward, Y leftward, Z backward for all leg

    Parameters
    ----------
    joint_position : numpy array (3)
        Array of the joint angles.
        [0] hips abduction revolute joint in RADIANS
        [1] hips flexion/extension revolute joint in RADIANS
        [2] knee flexion/extension revolute joint in RADIANS
    leg_index :
        Index of the current leg
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    config : [type]
        [description]

    Returns
    -------
    numpy array (3x3)
        J so that Foot XYZ Velocity in LRF = J x Joint rotation speed
    """      
    c1 = math.cos(joint_position[0])
    s1 = math.sin(joint_position[0])
    c2 = -math.sin(joint_position[1])
    s2 = math.cos(joint_position[1])
    c23 = math.sin(joint_position[1]+joint_position[2])
    s23 = -math.cos(joint_position[1]+joint_position[2])    
    return np.array(
        [
            [
                -s1*c23*config.LEG_LT - s1*c2*config.LEG_LF - c1*config.ABDUCTION_OFFSETS[leg_index],
                -c1*s23*config.LEG_LT - c1*s2*config.LEG_LF,
                -c1*s23*config.LEG_LT
            ],
            [
                 c1*c23*config.LEG_LT + c1*c2*config.LEG_LF - s1*config.ABDUCTION_OFFSETS[leg_index],
                -s1*s23*config.LEG_LT - s1*s2*config.LEG_LF,
                -s1*s23*config.LEG_LT
            ],
            [
                0,
                -c23*config.LEG_LT - c2*config.LEG_LF,
                -c23*config.LEG_LT
            ]
        ]
    )


def leg_inverse_kinematics_LRF(target_foot_position_LRF,initial_joint_position,leg_index,config,alpha=1.0):
    """Find the joint position from a foot position in the Leg Reference Frame

    LFR : centered HIPS axis, X upward, Y leftward, Z backward for all leg

    Parameters
    ----------
    target_foot_position_LRF : numpy array (3)
        Array of corresponding foot position in Leg Reference Frame
        [X,Y,Z] in LRF
    initial_joint_position : numpy array (3x4) 
        Array of the joint angles 
        [0] hips abduction revolute joint in RADIANS
        [1] hips flexion/extension revolute joint in RADIANS
        [2] knee flexion/extension revolute joint in RADIANS        
    leg_index :
        Index of the current leg
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    config : [type]
        [description]

    Returns
    -------
    numpy array (3)
        Array of the joint angles.
        [0] hips abduction revolute joint in RADIANS
        [1] hips flexion/extension revolute joint in RADIANS
        [2] knee flexion/extension revolute joint in RADIANS        
    """
    joint_position = initial_joint_position

    for counter in range(20):

        J = jacobian(joint_position,leg_index,config)
        #Jt = J.transpose()
        Jt = np.linalg.pinv(J)

        foot_position_LRF = leg_forward_kinematics_LRF(joint_position, leg_index, config)
        error = target_foot_position_LRF-foot_position_LRF
        step = alpha * (Jt @ error)
        joint_position += np.clip(step, -math.pi/4, math.pi/4)
        #joint_position = np.clip(joint_position,[-math.pi/2,0.0,0.0],[math.pi/2,1.2*math.pi,math.pi])
        if  np.linalg.norm(error) < 0.0001:
            break

    joint_position[1:2] = np.fmod(joint_position[1:2] + 2*np.pi, 2 * np.pi) 
    return joint_position 


def four_legs_inverse_kinematics_LRF(target_foot_position_LRF,initial_joint_position,config,alpha=1.0):
    """Find the position of the four feet in their respective Leg Reference Frame

    LFR : centered HIPS axis, X upward, Y leftward, Z backward for all leg

    Parameters
    ----------
    target_foot_position_LRF : numpy array (3x4) 
        Array of the foot position of four legs in LRF
        [0,i] X
        [1,i] Y
        [2,i] Z
    where i is the leg_index :
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    initial_joint_position : numpy array (3x4) 
        Array of the joint angles 
        [0,i] hips abduction revolute joint in RADIANS
        [1,i] hips flexion/extension revolute joint in RADIANS
        [2,i] knee flexion/extension revolute joint in RADIANS        
    where i is the leg_index :
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    config : [type]
        [description]

    Returns
    -------
    numpy array (3x4)
        Array of the joint angles.
        [0] hips abduction revolute joint in RADIANS
        [1] hips flexion/extension revolute joint in RADIANS
        [2] knee flexion/extension revolute joint in RADIANS        
    """
    joint_position = np.zeros((3,4))
    # for each leg
    for i in range(4):
        joint_position[:,i] = leg_inverse_kinematics_LRF(
            target_foot_position_LRF[:,i],
            initial_joint_position[:,i],
            i,
            config,
            alpha
        )
    return joint_position


def cos_law_c(a,b,angle):
    return math.sqrt(a**2+b**2-2*a*b*math.cos(angle))

def cos_law_A(a,b,c):
    x = (a**2+b**2-c**2) / (2*a*b)
    x = np.clip(x,-1.0,1.0)
    return math.acos( x )

# return joint position (coxa,hips,knee) in RADIANS from foot position (x,y,z) in METERS 
def leg_explicit_inverse_kinematics_LRF(foot_position_LRF,leg_index,config):
    log("foot_position:\n"+str(np.round(foot_position_LRF,3)))
    (x,y,z) = foot_position_LRF
    
    # hips to foot distance in XY plane
    CF_dist = (x ** 2 + y ** 2 ) ** 0.5
    log("CF dist: "+str(round(CF_dist,3)))
    
    CF_angle = math.atan2(-y,-x);
    log("CF angle: "+str(round(math.degrees(CF_angle),1))+"deg")

    # bound CF
    if False:
        CF_dist_min = math.sqrt(config.LEG_MIN_LENGTH**2+config.ABDUCTION_OFFSET**2)
        CF_dist_max =  math.sqrt(config.LEG_MAX_LENGTH**2+config.ABDUCTION_OFFSET**2)
        CF_dist = min(CF_dist_max,max(CF_dist,CF_dist_min))
        log("CF dist: "+str(round(CF_dist,3))+" bounded")

    # tibia+femur distance
    HF_dist_sq = CF_dist ** 2 - config.ABDUCTION_OFFSET ** 2
    HF_dist_sq = max(HF_dist_sq,0)
    HF_dist = HF_dist_sq ** 0.5
    log("HF: "+str(round(HF_dist,3)))

    # foot - coxa - hips angle
    FCH = cos_law_A(CF_dist,config.ABDUCTION_OFFSET,HF_dist)
    log("FCH: "+str(round(math.degrees(FCH),1))+"deg")

    # coxa angle
    HIPS_A = 0
    if leg_index == 1 or leg_index == 3:
        HIPS_A = CF_angle - FCH + math.pi/2.0
    if leg_index == 0 or leg_index == 2:
        HIPS_A = CF_angle + FCH - math.pi/2.0
    log("HIPS_A: "+str(round(math.degrees(HIPS_A),1))+"deg")

    # compute angle and elongation of the leg, in the (tibia/femur plane)
    L = (z ** 2 + HF_dist_sq) ** 0.5
    Alpha = math.atan2(HF_dist,-z);
    log("L: "+str(round(L,3)))
    log("Alpha: "+str(round(math.degrees(Alpha),0)))
    # bound max elongation
    if False:
        if(L>config.LEG_MAX_LENGTH):
            L = config.LEG_MAX_LENGTH
        if(L<config.LEG_MIN_LENGTH):
            L = config.LEG_MIN_LENGTH
        log("Lbounded: "+str(round(L,3)))   
      
    # compute angle from HIP to femur in coxa plane
    HIPS = Alpha + cos_law_A(L,config.LEG_LF,config.LEG_LT)
    log("HIPS: "+str(round(math.degrees(HIPS),0))+"deg")

    # compute angle from femur to tibia in coxa plance
    KNEE = cos_law_A(config.LEG_LF,config.LEG_LT,L)
    log("KNEE: "+str(round(math.degrees(KNEE),0))+"deg")
    joint_position = np.array([HIPS_A,HIPS,KNEE])
    joint_position[1:2] = np.fmod(joint_position[1:2] + 2*np.pi, 2 * np.pi)  
    return joint_position 


def four_legs_explicit_inverse_kinematics_LRF(target_foot_position_LRF,config):
    """Find the joint position of the four feet from their position in their respective Leg Reference Frame

    LFR : centered HIPS axis, X upward, Y leftward, Z backward for all leg

    Parameters
    ----------
    target_foot_position_LRF : numpy array (3x4) 
        Array of the foot position of four legs in LRF
        [0,i] X
        [1,i] Y
        [2,i] Z
    where i is the leg_index :
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    config : [type]
        [description]

    Returns
    -------
    numpy array (3x4)
        Array of the joint angles.
        [0] hips abduction revolute joint in RADIANS
        [1] hips flexion/extension revolute joint in RADIANS
        [2] knee flexion/extension revolute joint in RADIANS        
    """
    joint_position = np.zeros((3,4))
    # for each leg
    for i in range(4):
        joint_position[:,i] = leg_explicit_inverse_kinematics_LRF(
            target_foot_position_LRF[:,i],
            i,
            config
        )
    return joint_position



def four_legs_explicit_inverse_kinematics_BRF(target_feet_position_BRF,config):
    """Find the joint position of the four feet from their position in the Body Reference Frame

    LFR : centered HIPS axis, X upward, Y leftward, Z backward for all leg

    Parameters
    ----------
    target_feet_position_BRF : numpy array (3x4) 
        Array of the foot position of four legs in LRF
        [0,i] X
        [1,i] Y
        [2,i] Z
    where i is the leg_index :
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    config : [type]
        [description]

    Returns
    -------
    numpy array (3x4)
        Array of the joint angles.
        [0] hips abduction revolute joint in RADIANS
        [1] hips flexion/extension revolute joint in RADIANS
        [2] knee flexion/extension revolute joint in RADIANS        
    """
    joint_position = np.zeros((3,4))
    # for each leg
    for i in range(4):
        target_foot_position_LRF = R_BRF_to_LRF.dot(target_feet_position_BRF[:,i]-config.LEG_ORIGINS[:,i])

        joint_position[:,i] = leg_explicit_inverse_kinematics_LRF(
            target_foot_position_LRF,
            i,
            config
        )
    return joint_position



def four_legs_inverse_kinematics_BRF(target_feet_position_BRF,initial_joint_position,config,alpha=1.0):
    """Find the joint position of the four feet from their position in the Body Reference Frame

    LFR : centered HIPS axis, X upward, Y leftward, Z backward for all leg

    Parameters
    ----------
    target_feet_position_BRF : numpy array (3x4) 
        Array of the foot position of four legs in LRF
        [0,i] X
        [1,i] Y
        [2,i] Z
    where i is the leg_index :
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    initial_joint_position : numpy array (3x4) 
        Array of the joint angles 
        [0,i] hips abduction revolute joint in RADIANS
        [1,i] hips flexion/extension revolute joint in RADIANS
        [2,i] knee flexion/extension revolute joint in RADIANS        
    where i is the leg_index :
        0: Front Right
        1: Front Left
        2: Read Right
        3: Rear Left
    config : [type]
        [description]

    Returns
    -------
    numpy array (3x4)
        Array of the joint angles.
        [0] hips abduction revolute joint in RADIANS
        [1] hips flexion/extension revolute joint in RADIANS
        [2] knee flexion/extension revolute joint in RADIANS        
    """
    joint_position = np.zeros((3,4))
    # for each leg
    for i in range(4):
        target_foot_position_LRF = R_BRF_to_LRF.dot(target_feet_position_BRF[:,i]-config.LEG_ORIGINS[:,i])

        joint_position[:,i] = leg_inverse_kinematics_LRF(
            target_foot_position_LRF,
            initial_joint_position[:,i],
            i,
            config
        )
    return joint_position







    # T_LRF_to_BRF = np.array([
    #         [ 0, 0, -1,  config.LEG_FB],
    #         [ 0, 1,  0, -config.LEG_LR],
    #         [ 1, 0,  0, 0],
    #         [ 0, 0,  0, 1]
    #     ])

    # T_BRF_to_LRF = np.array([
    #         [ 0, 0, 1,  0],
    #         [ 0, 1,  0,  config.LEG_LR],
    #         [-1, 0,  0,  config.LEG_FB],
    #         [ 0, 0,  0, 1]
    #     ])

    # if False:
    #     print(T_LRF_to_BRF)
    #     print(T_BRF_to_LRF)


    # if False:
    #     F_LRF = np.array([config.default_z_ref,0,0,1])
    #     print(F_LRF)
    #     F_BRF = T_LRF_to_BRF.dot(F_LRF)
    #     print(F_BRF)

    # if False :
    #     F_BRF = np.append(default_stance_default_z[:,0],[1],0)
    #     print(F_BRF)
    #     F_LRF = T_BRF_to_LRF.dot(F_BRF)
    #     print(F_LRF)
    # if False :
    #     F_LRF = np.array([-0.1,-0.01,0.02,1])
    #     print(F_LRF)
    #     F_BRF = T_LRF_to_BRF.dot(F_LRF)
    #     print(F_BRF)
    #     F_LRF = T_BRF_to_LRF.dot(F_BRF)
    #     print(F_LRF)
