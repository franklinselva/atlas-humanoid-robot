"""PyBullet Simulator"""
from typing import Tuple
import pybullet as p

from .pybullet_api import PyBulletSimulator


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
        """Retrieve the robot."""
        if self._simulator.robot is None:
            raise Exception("Robot not loaded.")

        return self._simulator.robot

    @property
    def env_id(self) -> int:
        """Retrieve the environment."""
        return self._simulator.get_client_id()
