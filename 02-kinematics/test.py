from Config import Configuration
import Kinematics
import numpy as np
import math
import matplotlib.pyplot as plt


##DEBUG #######################################################################

# TRACE macro
DEBUG = True
#DEBUG = False

def log(s):
    if DEBUG:
        print(s)

##DEBUG #######################################################################


if __name__ == '__main__':

	config = Configuration()

	# test single forward kinematics
	# angles:
	# [  0. 180.  90.]
	# foot_LRF:
	# [-0.15 -0.06  0.14]
	if False:
		joint_position = np.radians(np.array([
			0.0,		# hips abduction revolute joints
			180.0,		# hips flexion/extension revolute joints
			90.0 		# knee flexion/extension revolute joints
		]))
		print("joint_position:\n"+str(np.round(np.degrees(joint_position),1)))
		foot_position_LRF = Kinematics.leg_forward_kinematics_LRF(joint_position, 1, config)
		print("foot_position_LRF:\n"+str(np.round(foot_position_LRF,3)))


	# test four leg forward kinematics
	# angles:
	# [[  0.   0.  90. -15.]
	#  [180.  90. 180. 125.]
	#  [ 90.  90.  90.  45.]]
	# feet_LRF:
	# [[-0.15  -0.14   0.06  -0.07 ]
	#  [-0.06   0.06  -0.15   0.081]
	#  [ 0.14  -0.15   0.14  -0.067]]
	if False:
		joint_position = np.radians(np.array(
			[	#   FR,   FL,   RR,   RL
				[  0.0,  0.0, 90.0,-15.0], # hips abduction revolute joints
				[180.0, 90.0,180.0,125.0], # hips flexion/extension revolute joints
				[ 90.0, 90.0, 90.0, 45.0]  # knee flexion/extension revolute joints
			]
		))
		print("joint_position:\n"+str(np.round(np.degrees(joint_position),1)))
		feet_LRF = Kinematics.four_legs_forward_kinematics_LRF(joint_position,config)
		print("feet_LRF:\n"+str(np.round(feet_LRF,3)))


	# test jacobian single leg
	if False:
		joint_pos = np.radians(np.array([
			10.0,		# hips abduction revolute joints
			180.0,		# hips flexion/extension revolute joints
			90.0 		# knee flexion/extension revolute joints
		]))
		print("joint_pos:\n"+str(np.round(np.degrees(joint_pos),1)))
		joint_vel = np.radians(np.array([
			-2.0,
			 0.0,
			-0.0
		]))
		print("joint_vel:\n"+str(np.round(np.degrees(joint_vel),1)))
		J = Kinematics.jacobian(joint_pos,0,config)
		print("J:\n"+str(np.round(J,3)))
		foot_pos_LRF = Kinematics.leg_forward_kinematics(joint_pos, 0, config)
		print("foot_pos_LRF:\n"+str(np.round(foot_pos_LRF,3)))		
		foot_vel_LRF = J @ joint_vel
		print("foot_vel_LRF:\n"+str(np.round(foot_vel_LRF,3)))


	# test jacobian & inverse kinemtics for a single leg
	# target_foot_position_LRF:
	# [-0.19 -0.06 -0.02]
	# joint_position:
	# [-30. 170. 110.]
	# 5
	# joint_pos:
	# [[  0.  135.1  82.3]]	
	if False:

		target_joint_position = np.radians(np.array([
			20.0,		# hips abduction revolute joints
			130.0,		# hips flexion/extension revolute joints
			80.0 		# knee flexion/extension revolute joints
		]))
		print("target_joint_position:\n"+str(np.round(np.degrees(target_joint_position),1)))

		target_foot_position_LRF = Kinematics.leg_forward_kinematics_LRF(target_joint_position,1,config)
		print("target_foot_position_LRF:\n"+str(np.round(target_foot_position_LRF,3)))

		initial_joint_position = np.radians(np.array([
			-30.0,		# hips abduction revolute joints
			170.0,		# hips flexion/extension revolute joints
			110.0 		# knee flexion/extension revolute joints
		]))
		print("initial_joint_position:\n"+str(np.round(np.degrees(initial_joint_position),1)))


		Ik_joint_position = Kinematics.leg_inverse_kinematics_LRF(target_foot_position_LRF,initial_joint_position,1,config)
		print("Ik_joint_position:\n"+str(np.round(np.degrees(Ik_joint_position),1)))


	# test inverse kinemtics for four legs
	if False:
		initial_joint_position = np.radians(np.array(
			[	#   FR,   FL,   RR,   RL
				[  0.0,  0.0,  0.0,  0.0], # hips abduction revolute joints
				[130.0,130.0,130.0,130.0], # hips flexion/extension revolute joints
				[ 90.0, 90.0, 90.0, 90.0]  # knee flexion/extension revolute joints
			]
		))
		print("initial_joint_position:\n"+str(np.round(np.degrees(initial_joint_position),1)))
		initial_feet_LRF = Kinematics.four_legs_forward_kinematics_LRF(initial_joint_position,config)
		print("initial_feet_LRF:\n"+str(np.round(initial_feet_LRF,3)))

		target_joint_position = np.radians(np.array(
			[	#   FR,   FL,   RR,   RL
				[ 25.0, 10.0,-33.0,-15.0], # hips abduction revolute joints
				[200.0,160.0,190.0, 95.0], # hips flexion/extension revolute joints
				[120.0, 80.0, 70.0, 45.0]  # knee flexion/extension revolute joints
			]
		))
		print("target_joint_position:\n"+str(np.round(np.degrees(target_joint_position),1)))
		target_feet_position_LRF = Kinematics.four_legs_forward_kinematics_LRF(target_joint_position,config)
		print("target_feet_position_LRF:\n"+str(np.round(target_feet_position_LRF,3)))

		IK_joint_position = Kinematics.four_legs_inverse_kinematics_LRF(target_feet_position_LRF,initial_joint_position,config)
		IK_explicit_joint_position = Kinematics.four_legs_explicit_inverse_kinematics_LRF(target_feet_position_LRF,config)

		print("target_joint_position:\n"+str(np.round(np.degrees(target_joint_position),1)))
		print("IK_joint_position:\n"+str(np.round(np.degrees(IK_joint_position),1)))
		print("IK_explicit_joint_position:\n"+str(np.round(np.degrees(IK_explicit_joint_position),1)))




	# test inverse explicit kinemtics for four legs in BRF
	if False:

		default_stance_default_z_BRF = config.default_stance + np.repeat(np.array([0.0,0.0,config.default_z_ref])[:,np.newaxis],4,1)
		default_stance_default_z_BRF = config.default_stance + np.array([[0.0,0.0,config.default_z_ref],]*4).transpose()
		print("default_stance_default_z:\n"+str(np.round(default_stance_default_z_BRF,3)))

		target_feet_position_BRF = default_stance_default_z_BRF

		IK_explicit_joint_position = Kinematics.four_legs_explicit_inverse_kinematics_BRF(target_feet_position_BRF,config)
		print("IK_explicit_joint_position:\n"+str(np.round(np.degrees(IK_explicit_joint_position),1)))



	# test inverse iterative kinemtics for four legs in BRF
	# test inverse iterative kinemtics for four legs in BRF
	# test inverse iterative kinemtics for four legs in BRF
	# test inverse iterative kinemtics for four legs in BRF

	if True:

		default_stance_default_z_BRF = config.default_stance + np.repeat(np.array([0.0,0.0,config.DEFAULT_Z])[:,np.newaxis],4,1)
		default_stance_default_z_BRF = config.default_stance + np.array([[0.0,0.0,config.DEFAULT_Z],]*4).transpose()
		print("default_stance_default_z:\n"+str(np.round(default_stance_default_z_BRF,3)))

		target_feet_position_BRF = default_stance_default_z_BRF

	print("Default standing pose:\n"+str(np.round(config.default_stance_with_z,3)))