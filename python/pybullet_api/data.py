"""Data API for PyBullet simulator."""
from typing import List
import pybullet as p  # type: ignore[import]


class PyBulletData:
    """Data API for PyBullet"""

    def __init__(
        self,
        model_id: int,
        physics_client: int,
    ) -> None:
        """Data handler for pybullet

        Args:
            model_id (int): robot id included in the simulation
            physics_client (int): The physics client ID for the simualtor
        """
        self._model_id = model_id
        self._physics_client = physics_client

        self._initial_pose = self.get_current_pose()
        self._num_joints = p.getNumJoints(
            self._model_id, physicsClientId=self._physics_client
        )
        self._joint_names = [
            p.getJointInfo(self._model_id, i, physicsClientId=self._physics_client)[
                1
            ].decode()
            for i in range(self.num_joints)
        ]

        self._joints_info = [
            p.getJointInfo(self._model_id, i, physicsClientId=self._physics_client)
            for i in range(self.num_joints)
        ]

    def get_current_pose(self) -> tuple:
        """Get the current pose of the robot

        Returns:
            tuple: Tuple containing the position and orientation (x, y, z, qx, qy, qz, qw)
        """
        position, orientation = p.getBasePositionAndOrientation(
            self._model_id, physicsClientId=self._physics_client
        )
        return tuple(position.extend(orientation))

    @property
    def initial_pose(self) -> tuple:
        """Get the initial pose of the object.

        Returns:
            tuple: Initial pose of the robot
        """
        return self._initial_pose

    @property
    def num_joints(self) -> int:
        """Number of joints.

        Returns:
            int: Number of joints in the robot model
        """
        return self._num_joints

    @property
    def joint_names(self) -> List[str]:
        """List of joint names.

        Returns:
            list[str]: List of Joint names in order of URDF
        """
        return self._joint_names

    @property
    def joints_info(
        self,
    ) -> List[tuple]:  # FIXME: List cannot be generic in <=Python3.8
        """List of joint info.

        Returns:
            list[tuple, int]: List of Joint information
        """
        return self._joints_info

    def get_joint_state(self, joint_name: str) -> tuple:
        """Get the joint state.

        Args:
            joint_name(str): Name of the joint
        Returns:
            tuple: Get joint state of the particular joint
        """
        assert joint_name in self._joint_names, f"{joint_name} not in joint names"

        joint_id = self.get_joint_id(joint_name)
        return p.getJointState(
            self._model_id, joint_id, physicsClientId=self._physics_client
        )

    def get_joint_id(self, joint_name: str) -> int:
        """Get the joint id.

        Args:
            joint_name(str): Name of the joint

        Returns:
            int: joint id of the particular joint
        """
        assert joint_name in self._joint_names, f"{joint_name} not in joint names"

        joint_id = self._joint_names.index(joint_name)
        return joint_id
