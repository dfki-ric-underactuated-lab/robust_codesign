"""
Abstract Controller
===================

Abstract controller class to which all controller classes have to adhere.
"""


from abc import ABC, abstractmethod


class AbstractController(ABC):
    """
    Abstract controller class. All controller should inherit from this abstract class.
    """
    def init(self, x0):
        """
        Initialize the controller. May not be necessary.

        Parameters
        ----------
        ``x0``: ``array like``
            The start state of the pendulum
        """
        self.x0 = x0

    def set_goal(self, x):
        """
        Set the desired state for the controller. May not be necessary.

        Parameters
        ----------
        ``x``: ``array like``
            The desired goal state of the controller
        """

        self.goal = x

@abstractmethod
def get_control_output(self, mea_cart_pos, mea_pend_pos, mea_cart_vel, mea_pend_vel, mea_force, mea_time):
    """
    The function to compute the control input for the cartpole actuator.
    Supposed to be overwritten by actual controllers.
    The API of this method should be adapted.
    Unused inputs/outputs can be set to None.

    **Parameters**

    ``mea_cart_pos``: ``float``
        The position of the cart [m]
    ``mea_pend_pos``: ``float``
        The position of the pendulum [rad]
    ``mea_cart_vel``: ``float``
        The velocity of the cart [m/s]
    ``mea_pend_vel``: ``float``
        The velocity of the pendulum [rad/s]
    ``mea_force``: ``float``
        The measured force on the cart [N]
    ``mea_time``: ``float``
        The collapsed time [s]

    Returns
    -------
    ``des_cart_pos``: ``float``
        The desired position of the cart [m]
    ``des_pend_pos``: ``float``
        The desired position of the pendulum [rad]
    ``des_cart_vel``: ``float``
        The desired velocity of the cart [m/s]
    ``des_pend_vel``: ``float``
        The desired velocity of the pendulum [rad/s]
    ``des_force``: ``float``
        The force supposed to be applied by the actuator [N]
    """

    des_cart_pos = None
    des_pend_pos = None
    des_cart_vel = None
    des_pend_vel = None
    des_force = None
    return des_cart_pos, des_pend_pos, des_cart_vel, des_pend_vel, des_force
