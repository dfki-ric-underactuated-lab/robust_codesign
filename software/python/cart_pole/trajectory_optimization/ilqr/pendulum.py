"""
Pendulum Dynamics
=================
"""

import numpy as np
import sympy as smp


def discrete_dynamics_euler(sys, x, u, dt):
    x_d = sys.continuous_dynamics3(x, u)
    x_next = x + x_d * dt
    return x_next


def discrete_dynamics_rungekutta(sys, x, u, dt):
    k1 = sys.continuous_dynamics(x, u)
    k2 = sys.continuous_dynamics(x+0.5*dt*k1, u)
    k3 = sys.continuous_dynamics(x+0.5*dt*k2, u)
    k4 = sys.continuous_dynamics(x+dt*k3, u)
    x_d = (k1 + 2 * (k2 + k3) + k4) / 6.0
    x_next = tuple(x + x_d*dt)
    return x_next


def swingup_stage_cost(x, u, goal=[0, 0, 0, 0], Cu=0.1, Cpc=0.1, Cpp=0.1, Cvc=0.1, Cvp=0.1):
    # eps = 1e-6
    c_control = u[0] ** 2
    c_pos_cart = (x[0] - goal[0]) ** 2.0
    # c_pos_pend = (((x[1] + 1) % (2 * 1) - 1) - goal[1] + eps) ** 2.0
    c_pos_pend = (x[1] - goal[1]) ** 2.0
    c_vel_cart = (x[2] - goal[2]) ** 2.0
    c_vel_pend = (x[3] - goal[3]) ** 2.0
    # maybe include energy
    return Cu * c_control + Cpc * c_pos_cart + Cpp * c_pos_pend + Cvc * c_vel_cart + Cvp * c_vel_pend
    # D = x[1] % 2
    # return D

def swingup_final_cost(x, goal=[0, 0, 0, 0], Cpc=10.0, Cpp=1000.0, Cvc=1.0, Cvp=1.0):
    # eps = 1e-6
    # pp = (x[1] + 1) % (2 * 1) - 1
    c_pos_cart = (x[0] - goal[0]) ** 2.0
    c_pos_pend = (x[1] - goal[1]) ** 2.0
    c_vel_cart = (x[2] - goal[2]) ** 2.0
    c_vel_pend = (x[3] - goal[3]) ** 2.0
    # maybe include energy
    return Cpc * c_pos_cart + Cpp * c_pos_pend + Cvc * c_vel_cart + Cvp * c_vel_pend
