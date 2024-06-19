import roboticstoolbox as rtb
import numpy as np
import swift
import roboticstoolbox as rtb
import spatialmath as sm

robot = rtb.models.Panda()
env = swift.Swift()
env.launch(realtime=True)

panda = rtb.models.Panda()
panda.q = panda.qr

Tep = panda.fkine(panda.q) * sm.SE3.Trans(0.2, 0.2, 0.45)

arrived = False
env.add(panda)

dt = 0.01

dataset = np.loadtxt("../../../devel/lib/advanced_robotics_franka_controllers/debug.txt")
q_set = dataset[:,0:7]
min_dist_set = dataset[:,7]
mani_set = dataset[:,8]
print(dataset.shape)

for i in range(q_set.shape[0]):
    panda.q = q_set[i,:]
    env.step(dt)