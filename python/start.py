import math, time
import pybullet as p
from pybullet_api.api import PyBulletSimulator
from pybullet_api.data import PyBulletData

def startup_env():
	PyBulletSimulator()
	PyBulletSimulator.load_robot(PyBulletSimulator)
	PyBulletSimulator.load_env(PyBulletSimulator)

	while(1):
		PyBulletSimulator.sim_step()  # done to keep the simulator alive


def main():
	startup_env()


if __name__ == "__main__":
	main()