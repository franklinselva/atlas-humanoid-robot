"""Entrypoint for the script."""
import os
import sys
import time

import pybullet as p

from python.env import PyBulletEnv

DATA_PATH = os.path.join(os.path.dirname(__file__), "..", "data")
ROBOT_PATH = "atlas/atlas_v4_with_multisense.urdf"
ENV_PATH = os.path.join(os.path.dirname(__file__), "..", "data", "botlab", "botlab.sdf")


def setup_env():
    """Setup the environment."""
    # TODO: In emtpy environment, the robot falls over. Why?
    # The behavior is expected but was not observed in the original code.
    if "empty" in sys.argv:
        env = PyBulletEnv(
            data_path=DATA_PATH,
            robot_path=ROBOT_PATH,
            robot_base_position=(0, 0, 0.9),
            env_path="plane.urdf",
        )
        p.resetDebugVisualizerCamera(
            cameraDistance=1.0,
            cameraYaw=91.8094,
            cameraPitch=-16.1578,
            cameraTargetPosition=[1.5079, 0.0, 0.28],
            physicsClientId=env.env_id,
        )

    else:
        # TODO: The botlab environment is not loaded correctly.
        env = PyBulletEnv(
            data_path=DATA_PATH,
            robot_path=ROBOT_PATH,
            robot_base_position=(-2, 3, -0.5),
            env_path=ENV_PATH,
        )
        p.resetDebugVisualizerCamera(
            cameraDistance=1,
            cameraYaw=148,
            cameraPitch=-9,
            cameraTargetPosition=[0.36, 5.3, -0.62],
            physicsClientId=env.env_id,
        )

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    p.getCameraImage(320, 200)  # , renderer=p.ER_BULLET_HARDWARE_OPENGL )
    t = 0

    while True:
        p.setGravity(0, 0, -10)
        time.sleep(0.01)
        t += 0.01
        keys = p.getKeyboardEvents()
        if keys.get(ord("q"), 0) & p.KEY_WAS_TRIGGERED:
            break
        else:
            p.stepSimulation()


def main():
    """Main function."""
    setup_env()


if __name__ == "__main__":
    main()
