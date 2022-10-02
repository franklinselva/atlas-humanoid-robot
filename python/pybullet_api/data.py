"""Data API for PyBullet simulator."""
import pybullet as p  # type: ignore[import]


class PyBulletData:
    """Data API for PyBullet"""

    def __init__(
        self,
        model_id: int,
        physics_client: int,
    ) -> None:
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
        """Get the current pose of the object."""
        position, orientation = p.getBasePositionAndOrientation(
            self._model_id, physicsClientId=self._physics_client
        )
        return tuple(position.extend(orientation))

    @property
    def initial_pose(self):
        """Get the initial pose of the object."""
        return self._initial_pose

    @property
    def num_joints(self):
        """Number of joints."""
        return self._num_joints

    @property
    def joint_names(self):
        """List of joint names."""
        return self._joint_names

    @property
    def joints_info(self):
        """List of joint info."""
        return self._joints_info

    def get_joint_state(self, joint_name: str) -> tuple:
        """Get the joint state."""
        joint_id = self.get_joint_id(joint_name)
        return p.getJointState(
            self._model_id, joint_id, physicsClientId=self._physics_client
        )

    def get_joint_id(self, joint_name: str) -> int:
        """Get the joint id."""
        joint_id = self._joint_names.index(joint_name)
        return joint_id
