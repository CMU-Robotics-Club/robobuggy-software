from abc import ABC, abstractmethod
import sys

from nav_msgs.msg import Odometry


sys.path.append("/rb_ws/src/buggy/buggy")
from util.trajectory import Trajectory

class Controller(ABC):
    """
    Base class for all controllers.

    The controller takes in the current state of the buggy and a reference
    trajectory. It must then compute the desired control output.

    The method that it does this by is up to the implementation, of course.
    Example schemes include Pure Pursuit, Stanley, and LQR.
    """

    # TODO: move this to a constants class
    NAND_WHEELBASE = 1.3
    SC_WHEELBASE = 1.104

    def __init__(self, start_index: int, namespace : str, node) -> None:
        self.namespace = namespace
        if namespace.upper() == '/NAND':
            Controller.WHEELBASE = self.NAND_WHEELBASE
        else:
            Controller.WHEELBASE = self.SC_WHEELBASE

        self.current_traj_index = start_index
        self.node = node

    @abstractmethod
    def compute_control(
        self, state_msg: Odometry, trajectory: Trajectory,
    ) -> float:
        """
        Computes the desired control output given the current state and reference trajectory

        Args:
            state: (Odometry): state of the buggy, including position, attitude and associated twists
            trajectory (Trajectory): reference trajectory

        Returns:
            float (desired steering angle)
        """
        raise NotImplementedError