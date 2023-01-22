import matplotlib.pyplot as plt
import numpy as np

from pytransform3d.urdf import UrdfTransformManager
from pytransform3d.transformations import plot_transform
from pytransform3d.plot_utils import make_3d_axis


tm = UrdfTransformManager()
with open('./felin.urdf', "r") as f:
    tm.load_urdf(f.read())

for joint in tm._joints.keys():
	print(joint)

joint_position = np.radians(np.array(
    [   #   FR,   FL,   RR,   RL
        [  0.0,  0.0, 25.0,-15.0], # hips abduction revolute joints
        [180.0, 90.0,125.0,180.0], # hips flexion/extension revolute joints
        [ 90.0, 90.0, 45.0,170.0]  # knee flexion/extension revolute joints
    ]
))

tm.set_joint("FR-ABD", joint_position[0,0])
tm.set_joint("FL-ABD", joint_position[0,1])
tm.set_joint("RR-ABD", joint_position[0,2])
tm.set_joint("RL-ABD", joint_position[0,3])

tm.set_joint("FR-HIPS", joint_position[1,0])
tm.set_joint("FL-HIPS", joint_position[1,1])
tm.set_joint("RR-HIPS", joint_position[1,2])
tm.set_joint("RL-HIPS", joint_position[1,3])

tm.set_joint("FR-KNEE", joint_position[2,0])
tm.set_joint("FL-KNEE", joint_position[2,1])
tm.set_joint("RR-KNEE", joint_position[2,2])
tm.set_joint("RL-KNEE", joint_position[2,3])

legs = ["FR","FL","RR","RL"]
feet_BRF = np.zeros((3,4))
for leg_index in range(4):
	feet_BRF[:,leg_index] = tm.get_transform('%s-foot' % legs[leg_index], 'body-base')[0:3,3]
print("feet_BRF:\n\n"+str(np.round(feet_BRF,3))+"\n")


fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax = tm.plot_connections_in("felin", ax=ax)
tm.plot_visuals("felin", ax_s=0.3, alpha=1.0)
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
plt.show()