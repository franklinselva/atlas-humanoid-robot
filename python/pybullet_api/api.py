"""PyBullet Simulator API."""
import os
from typing import Optional, Tuple

import pybullet as p  # type: ignore[import]
import pybullet_data  # type: ignore[import]


class PyBulletSimulator:
    """PyBullet API"""

    GUI = p.GUI
    NO_GUI = p.DIRECT

    _robot = None

    def __init__(
        self,
        data_path: str = "",
        visualize: bool = True,
        initial_pose: Tuple[float, float, float, float, float, float, float] = (
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ),
        respawn: bool = True,
        real_time: bool = False,
    ) -> None:

        self._initial_pose = initial_pose
        self._respawn = respawn

        if visualize:
            self._physics_client = p.connect(self.GUI)
        else:
            self._physics_client = p.connect(self.NO_GUI)

        data_path = os.path.abspath(data_path)

        if os.path.exists(data_path):
            p.setAdditionalSearchPath(data_path)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self._real_time = real_time
        if real_time:
            p.setRealTimeSimulation(1, self._physics_client)

    def disconnect(self) -> None:
        """Disconnect from the physics server."""
        p.disconnect(self._physics_client)

    def set_gravity(self, gravity: float = -9.8) -> None:
        """Set the gravity."""
        p.setGravity(0, 0, -gravity, self._physics_client)

    def get_client_id(self) -> int:
        """get the client id"""
        return self._physics_client

    def set_time_step(self, time_step: float = 0.01) -> None:
        """Set the time step."""
        p.setTimeStep(time_step, self._physics_client)

    @property
    def robot(self) -> int:
        """Return the robot id."""
        if self._robot is None:
            raise Exception("Robot not loaded.")

        return self._robot

    @robot.setter
    def robot(self, robot_id: int) -> None:
        """Set the robot id."""
        self._robot = robot_id

    def load_robot(
        self,
        robot_path: "/Users/vigneshbalaji/atlas-humanoid-robot/data/atlas/atlas_v4_with_multisense.urdf",
        base_position: tuple = (-2,3,-0.5), # (0, 0, 0),
        base_orientation: tuple = (0, 0, 0, 1),
    ) -> Optional[int]:
        """Load a robot from a URDF file."""
        self._robot = p.loadURDF(
            robot_path,
            base_position,
            base_orientation,
            useFixedBase=False,
            flags=p.URDF_USE_SELF_COLLISION
            | p.URDF_USE_INERTIA_FROM_FILE
            | p.URDF_MERGE_FIXED_LINKS
            | p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
            | p.URDF_MAINTAIN_LINK_ORDER,
            globalScaling=1,
            physicsClientId=self._physics_client,
        )

        return self._robot

    def load_env(
        self,
        env_path: "/Users/vigneshbalaji/atlas-humanoid-robot/data/botlab/botlab.sdf",
        globalScaling: int = 2.0,
    ) -> Optional[int]:
        """Load a robot from a URDF file."""
        self._env = p.loadURDF(
            env_path,
            globalScaling,
        )

        return self._env



    def save_world(self, file_path: str) -> None:
        """Save the world to a file."""
        p.saveWorld(file_path, self._physics_client)

    def step(self) -> None:
        """Step the simulation."""
        if self._real_time:
            p.setRealTimeSimulation(1, self._physics_client)
        else:
            p.stepSimulation(physicsClientId=self._physics_client)

    def stop_simulation(self) -> None:
        """Stop the simulation."""
        p.disconnect(self._physics_client)

    def reset_simulation(self) -> None:
        """Reset the simulation."""
        p.resetSimulation(physicsClientId=self._physics_client)

    def reset_robot(self) -> None:
        """Reset the robot."""
        p.resetBasePositionAndOrientation(
            self._robot,
            self._initial_pose[:3],
            self._initial_pose[3:],
            physicsClientId=self._physics_client,
        )
