import math
import time

import pybullet as p


def setup_env():
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    atlas = p.loadURDF("atlas/atlas_v4_with_multisense.urdf", [-2, 3, -0.5])
    for i in range(p.getNumJoints(atlas)):
        p.setJointMotorControl2(atlas, i, p.POSITION_CONTROL, 0)
        print(p.getJointInfo(atlas, i))

    objs = p.loadSDF("botlab/botlab.sdf", globalScaling=2.0)
    zero = [0, 0, 0]
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    print("converting y to z axis")
    for o in objs:
        pos, orn = p.getBasePositionAndOrientation(o)
        y2x = p.getQuaternionFromEuler([3.14 / 2.0, 0, 3.14 / 2])
        newpos, neworn = p.multiplyTransforms(zero, y2x, pos, orn)
        p.resetBasePositionAndOrientation(o, newpos, neworn)

    p.loadURDF("boston_box.urdf", [-2, 3, -2], useFixedBase=True)

    p.resetDebugVisualizerCamera(
        cameraDistance=1,
        cameraYaw=148,
        cameraPitch=-9,
        cameraTargetPosition=[0.36, 5.3, -0.62],
    )

    p.loadURDF("boston_box.urdf", [0, 3, -2], useFixedBase=True)

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    p.getCameraImage(320, 200)  # , renderer=p.ER_BULLET_HARDWARE_OPENGL )

    t = 0
    p.setRealTimeSimulation(1)
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
