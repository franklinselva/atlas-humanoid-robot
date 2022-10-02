import math
import os
import time

import pybullet as p

from pybullet_api.api import PyBulletSimulator

maxForce = 500

DATA_PATH = os.path.join(os.path.dirname(__file__), "..", "data")


def startup_env():
    atlas_env = PyBulletSimulator(data_path=DATA_PATH, visualize=True)
    atlas_env.load_env(env_path="plane.urdf")
    atlas_env.load_robot(
        robot_path="atlas/atlas_v4_with_multisense.urdf", base_position=(0, 0, 0.0)
    )
    env_id = atlas_env.get_client_id()
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # debug configure, robot in view

    num_joints = p.getNumJoints(env_id)
    zero_vec = [1.5] * num_joints
    position = [0.2] * num_joints
    position[0] = 0.0
    position[8] = 1.5
    position[17] = 3.0
    position[29] = 0.5
    position[23] = -2.0
    position[25] = -1.0

    # 	for i in range(30):
    # 		print("Joint info index", i, p.getJointInfo(env_id, i))

    while 1:
        PyBulletSimulator.test_step()  # done to keep the simulator alive
        # p.setJointMotorControl2(bodyUniqueId=env_id, jointIndex=20, controlMode=p.VELOCITY_CONTROL, force = maxForce)
        p.setJointMotorControlArray(
            env_id,
            range(num_joints),
            p.POSITION_CONTROL,
            targetPositions=position,
            targetVelocities=zero_vec,
            positionGains=[1.0] * num_joints,
            velocityGains=[0.3] * num_joints,
        )


def setJointPosition(robot, position, kp=1.0, kv=0.3):
    num_joints = p.getNumJoints(robot)


def main():
    startup_env()


if __name__ == "__main__":
    main()
