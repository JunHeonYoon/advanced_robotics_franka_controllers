from math import pi
import numpy as np
from scipy.spatial.transform import Rotation as R

# 8-shaped path with 0.2m radius (along YZ-axis)
r = 0.2
t = np.linspace(0, 2.5*pi, 100) + pi/2
z = np.zeros(t.shape)
y = r * np.sin(2 * t)
x = r * np.sin(t)

# x = r * np.sin(t)
# y = r * np.sin(2 * t)
# z = r * np.cos(t)


# stayy initial orientation
rot = R.from_matrix([[1,  0,  0],
                     [0, -1, 0],
                     [0,  0, -1]])




quat = rot.as_quat()
quat_list = np.tile(quat, (t.size, 1))

total_desc = f""" 
{{
    "X": {list(map(np.double, x))},
    "Y": {list(map(np.double, y))},
    "Z": {list(map(np.double, z))},
    "quat_X": {list(map(np.double, quat_list[:,0]))},
    "quat_Y": {list(map(np.double, quat_list[:,1]))},
    "quat_Z": {list(map(np.double, quat_list[:,2]))},
    "quat_W": {list(map(np.double, quat_list[:,3]))}
}} 
"""

open('track.json','w').write(total_desc)
