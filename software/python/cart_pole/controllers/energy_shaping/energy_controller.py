"""
Energy Shaping Controller
==============
"""

# Other imports
import numpy as np

# Local imports
from controllers.abstract_controller import AbstractController


class EnergyShaping(AbstractController):
    """
    Controller which swings up the pole.
    """
    def __init__(self, sys):
        """
        Controller which swings up the pole.

        Parameters
        ----------
        sys : float, default=1.0
            mass of the pendulum [kg]
        """
        self.sys = sys

        if self.sys.selection == "short":
            self.k = -7
        elif self.sys.selection == "long":
            self.k = -10

    def set_goal(self, xG):
        pass

    def get_control_output(self, mea_time, mea_cart_pos, mea_pend_pos, mea_cart_vel, mea_pend_vel):
        """
        The function to compute the control input for the pendulum actuator

        Parameters
        ----------
        mea_time : float, default=None
            the collapsed time [s]
        mea_cart_pos : float
            the position of the cart [m]
            (not used)
        mea_pend_pos : float
            the position of the pendulum [rad]
        mea_cart_vel : float
            the velocity of the cart [m/s]
            (not used)
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

        des_force = self.k * (0.5 * self.sys.Jp * mea_pend_vel**2 + self.sys.Mp * self.sys.g * self.sys.lp *
                              (np.cos(mea_pend_pos) - 1)) * (2 * ((np.cos(mea_pend_pos) * mea_pend_vel) >= 0) - 1)

        # since this is a pure force controller, set desired states to 0
        des_cart_pos = None
        des_pend_pos = None
        des_cart_vel = None
        des_pend_vel = None

        return des_force
