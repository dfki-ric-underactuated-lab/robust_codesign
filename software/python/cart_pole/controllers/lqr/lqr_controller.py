"""
LQR Controller
==============
"""

# Other imports
import numpy as np

# Local imports
from controllers.lqr.lqr import lqr
from controllers.abstract_controller import AbstractController


class LQRController(AbstractController):
    """
    Controller which stabilizes the cartpole at its instable fixpoint.
    """
    def __init__(self, sys):
        """
        Controller which stabilizes the cartpole at its instable fixpoint.

        Parameters
        ----------
        sys : float, default=1.0
            mass of the pendulum [kg]
        """
        self.sys = sys

        [self.A, self.B, self.C, self.D] = self.sys.statespace()  # cartpole state space model linearized at theta = 0°

        if self.sys.selection == "short":
            # self.Q = np.diag([1, 6, 0, 0])  # selection of Q gains
            # self.R = np.array([0.0004])[:, np.newaxis]  # selection of R gains
            # test
            self.Q = np.diag([2500, 15000, 0, 0])  # selection of Q gains
            self.R = np.array([1])[:, np.newaxis]  # selection of R gains
        elif self.sys.selection == "long":
            self.Q = np.diag([1, 6, 0, 0])  # selection of Q gains
            self.R = np.array([0.0004])[:, np.newaxis]  # selection of R gains

        [self.K, self.S, self.eigVals] = lqr(self.A, self.B, self.Q, self.R)

    def set_goal(self, xG):
        self.xG = xG

    def get_control_output(self, mea_cart_pos, mea_pend_pos, mea_cart_vel, mea_pend_vel, mea_force=0, meas_time=0):
        """
        The function to compute the control input for the pendulum actuator

        Parameters
        ----------
        mea_cart_pos : float
            the position of the cart [m]
        mea_pend_pos : float
            the position of the pendulum [rad]
        mea_cart_vel : float
            the velocity of the cart [m/s]
        mea_pend_vel : float
            the velocity of the pendulum [rad/s]
        mea_force : float, default=0
            the measured force of the pendulum [N]
            (not used)
        meas_time : float, default=0
            the collapsed time [s]
            (not used)

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

        force = np.matmul(self.K, (self.xG - np.array([mea_cart_pos, mea_pend_pos, mea_cart_vel, mea_pend_vel])))

        # since this is a pure force controller, set desired states to 0
        # des_cart_pos = None
        # des_pend_pos = None
        # des_cart_vel = None
        # des_pend_vel = None

        return force
