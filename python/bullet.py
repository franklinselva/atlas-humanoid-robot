"""PyBullet Simulator"""
from .pybullet_api import PyBulletSimulator


class PyBullet:
    """PyBullet Simulator"""

    def __init__(
        self,
        data_path: str = "",
        robot_path: str = "",
        visualize: bool = True,
    ) -> None:
        # Setup Simulator
        self._simulator = PyBulletSimulator(data_path=data_path, visualize=visualize)
        self._simulator.reset_simulation()
        self._simulator.set_gravity()
        self._simulator.set_time_step()

        # Load Robot
        if robot_path != "":
            self._robot = self._simulator.load_robot(robot_path=robot_path)

    def stop_simulation(self) -> None:
        """Stop the simulation."""
        self._simulator.stop_simulation()

    @property
    def robot(self) -> int:
        """Retrieve the robot."""
        if self._simulator.robot is None:
            raise Exception("Robot not loaded.")

        return self._simulator.robot
