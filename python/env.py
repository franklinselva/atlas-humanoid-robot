"""PyBullet Simulator"""
from typing import Tuple

import pybullet as p

from .pybullet_api import PyBulletData, PyBulletSimulator

# TODO: Integrate pybullet_data into the class definition.


class PyBulletEnv:
    """PyBullet Simulator"""

    def __init__(
        self,
        data_path: str = "",
        robot_path: str = "",
        robot_base_position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        env_path: str = "",
        fixed_robot_base: bool = False,
        visualize: bool = True,
    ) -> None:
        """Pybullet environment

        Args:
            data_path (str, optional): Additional data path for pybullet. Defaults to "".
            robot_path (str, optional): file path to the robot urdf. Defaults to "".
            robot_base_position (Tuple[float, float, float], optional):
                    Position of the robot base (x, y, z). Defaults to (0.0, 0.0, 0.0).
            env_path (str, optional): file path to the environment sdf/world. Defaults to "".
            fixed_robot_base (bool, optional): The robot base will be fixed if true.
                    Defaults to False.
            visualize (bool, optional): Visualize the environment with GUI. Defaults to True.
        """
        # Setup Simulator
        self._simulator = PyBulletSimulator(data_path=data_path, visualize=visualize)
        self._simulator.reset_simulation()
        self._simulator.set_gravity()
        self._simulator.set_time_step()

        # Load Robot
        if visualize:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

        if robot_path != "":

            self._robot = self._simulator.load_robot(
                robot_path=robot_path,
                fixed_base=fixed_robot_base,
                base_position=robot_base_position,
            )

        if fixed_robot_base:
            self._start_gui_control()

        # Load Environment
        if env_path != "":
            self._env = self._simulator.load_env(env_path=env_path)

        if visualize:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    def stop_simulation(self) -> None:
        """Stop the simulation."""
        self._simulator.stop_simulation()

    @property
    def robot(self) -> int:
        """Retrieve the robot.

        Returns:
            int: The robot ID
        """
        if self._simulator.robot is None:
            raise Exception("Robot not loaded.")

        return self._simulator.robot

    @property
    def env_id(self) -> int:
        """Retrieve the environment.

        Returns:
            int: The environment ID
        """
        return self._simulator.get_client_id()

    def _start_gui_control(self) -> None:
        """Start GUI control of the robot."""
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)

        # TODO: Add support for gui based joint control when using fixed base
