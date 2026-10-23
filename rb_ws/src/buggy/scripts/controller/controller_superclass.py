from abc import ABC, abstractmethod

from nav_msgs.msg import Odometry

from util.trajectory import Trajectory
from util.constants import Constants

class Controller(ABC):
    """
    Base class for all controllers.

    The controller takes in the current state of the buggy and a reference
    trajectory. It must then compute the desired control output.

    The method that it does this by is up to the implementation, of course.
    Example schemes include Pure Pursuit, Stanley, and LQR.
    """

    def __init__(self, start_index: int, namespace : str, node) -> None:
        self.namespace = namespace
        if namespace.upper() == '/SC':
            Controller.WHEELBASE = Constants.WHEELBASE_SC
        elif namespace.upper() == '/NAND':
            Controller.WHEELBASE = Constants.WHEELBASE_NAND
        else:
            Controller.WHEELBASE = Constants.WHEELBASE_SC
            node.get_logger().error("WARNING: NAMESPACE NOT RECOGNIZED: " + namespace + ". DEFAULTING TO SC WHEELBASE")

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