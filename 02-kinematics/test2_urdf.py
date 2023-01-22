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

tm.set_joint("FR-ABD",  3 * np.pi / 180.0)
tm.set_joint("RR-ABD",  3 * np.pi / 180.0)
tm.set_joint("FL-ABD", -3 * np.pi / 180.0)
tm.set_joint("RL-ABD", -3 * np.pi / 180.0)

tm.set_joint("FR-HIPS", 145 * np.pi / 180.0)
tm.set_joint("RR-HIPS", 145 * np.pi / 180.0)
tm.set_joint("FL-HIPS", 145 * np.pi / 180.0)
tm.set_joint("RL-HIPS", 145 * np.pi / 180.0)

tm.set_joint("FR-KNEE",  85 * np.pi / 180.0)
tm.set_joint("RR-KNEE",  85 * np.pi / 180.0)
tm.set_joint("FL-KNEE",  85 * np.pi / 180.0)
tm.set_joint("RL-KNEE",  85 * np.pi / 180.0)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax = tm.plot_connections_in("felin", ax=ax)
tm.plot_visuals("felin", ax_s=0.3, alpha=1.0)
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
plt.show()