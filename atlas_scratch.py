import math, time
import pybullet as p

#import python.pybullet_api
#from python.pybullet_api import PyBulletSimulator, PyBulletData
from python.pybullet_api.api import PyBulletSimulator
from python.pybullet_api.data import PyBulletData


def startup_env():
	PyBulletSimulator()

	atlas = p.loadURDF("/Users/vigneshbalaji/atlas-humanoid-robot/data/atlas/atlas_v4_with_multisense.urdf", [-2,3,-0.5])

	while(1):
		p.getConnectionInfo()



def main():
	startup_env()


if __name__ == "__main__":
	main()
