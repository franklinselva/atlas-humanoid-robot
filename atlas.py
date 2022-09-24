import pybullet as p
import time
p.connect(p.GUI)
atlas = p.loadURDF("/Users/vigneshbalaji/atlas-humanoid-robot/atlas_v4_with_multisense.urdf", [-2,3,-0.5])


p.disconnect()
