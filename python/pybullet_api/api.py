"""PyBullet Simulator API."""
import os
from typing import Optional, Tuple

import pybullet as p  # type: ignore[import]
import pybullet_data  # type: ignore[import]


class PyBulletSimulator:
    """PyBullet Simulator API"""

    GUI = p.GUI
    NO_GUI = p.DIRECT

    _robot = None
    _env = None

    def __init__(
        self,
        data_path: str,
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
        """Pybullet Simulator API

        Args:
            data_path (str): Additional data path for pybullet.
            visualize (bool, optional): Visualize the simulator. Defaults to True.
            initial_pose (Tuple[float, float, float, float, float, float, float], optional):
                Initial pose of the robot base. Defaults to ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, ).
            respawn (bool, optional): Respawn the robot and the environment. Defaults to True.
            real_time (bool, optional): Set the simualtion to realtime. Defaults to False.
        """

        self._initial_pose = initial_pose
        self._respawn = respawn

        if visualize:
            self._physics_client = p.connect(self.GUI)
        else:
            self._physics_client = p.connect(self.NO_GUI)

        data_path = os.path.abspath(data_path)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        if os.path.exists(data_path):
            p.setAdditionalSearchPath(data_path)

        self._real_time = real_time
        if real_time:
            p.setRealTimeSimulation(1, self._physics_client)

    def disconnect(self) -> None:
        """Disconnect from the physics server."""
        p.disconnect(self._physics_client)

    def set_gravity(self, gravity: float = -9.8) -> None:
        """Set gravity for the environment

        Args:
            gravity (float, optional): Gravity value to be set along z-axis. Defaults to -9.8.
        """
        p.setGravity(0, 0, -gravity, self._physics_client)

    def get_client_id(self) -> int:
        """Get the physics client id of the GUI window

        Returns:
            int: pybullet physics client ID
        """
        return self._physics_client

    def set_time_step(self, time_step: float = 0.01) -> None:
        """Set time step for the simulation

        Args:
            time_step (float, optional): Time step in seconds. Defaults to 0.01.
        """
        p.setTimeStep(time_step, self._physics_client)

    @property
    def robot(self) -> int:
        """Robot imported to the environment

        ### Note:
         - Only one robot can be currently imported to the environment
         #TODO: Update the pybullet simulator to support multi-robot environment

        Raises:
            Exception: If the robot is not loaded prior to the call

        Returns:
            int: The robot ID
        """
        if self._robot is None:
            raise Exception("Robot not loaded.")

        return self._robot

    @robot.setter
    def robot(self, robot_id: int) -> None:
        """Assign the robot ID

        Args:
            robot_id (int): The robot ID loaded to the environment
        """
        self._robot = robot_id

    def load_robot(
        self,
        robot_path: str,
        base_position: tuple = (0, 0, 0),
        base_orientation: tuple = (0, 0, 0, 1),
        fixed_base: bool = False,
    ) -> Optional[int]:
        """Load the robot to the environment

        Args:
            robot_path (str): urdf path to the robot
            base_position (tuple, optional): Position of the robot base (x, y, z).
                    Defaults to (0, 0, 0).
            base_orientation (tuple, optional): Orientation of the robt base (qx, qy, qz, qw).
                    Defaults to (0, 0, 0, 1).
            fixed_base (bool, optional): Spawn the robot with fixed base if true.
                    Defaults to False.

        Returns:
            Optional[int]: The robot ID within the simulation
        """
        self._robot = p.loadURDF(
            robot_path,
            base_position,
            base_orientation,
            useFixedBase=fixed_base,
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
        env_path: str = "plane.urdf",
    ) -> Optional[int]:
        """Load environment into the simulator

        Args:
            env_path (str, optional): SDF/world file path to the environment.
                    Defaults to "plane.urdf".

        Returns:
            Optional[int]: The environment ID within the simulation
        """
        self._env = p.loadSDF(
            env_path,
            globalScaling=2.0,
        )

        return self._env

    def save_world(self, file_path: str) -> None:
        """Save the world to a file

        Args:
            file_path (str): File path to save the world
        """
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
