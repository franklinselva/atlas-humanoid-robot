import math, time
#import python.pybullet_api
#from python.pybullet_api import PyBulletSimulator, PyBulletData
from python.pybullet_api.api import PyBulletSimulator
from python.pybullet_api.data import PyBulletData


def startup_env():
	PyBulletSimulator()
#	while(1):                # write a step function, example just defining simulation timestep 
#		print("good")



def main():
	startup_env()


if __name__ == "__main__":
	main()
