"""
Parameters
==========
"""

# Global imports
# import math
# import yaml
import numpy as np
from numpy import sin,cos
import sympy as smp
import pydrake.symbolic as sym 
from jinja2 import Environment, FileSystemLoader

class Cartpole:
    def __init__(self, selection):
        """
        **Organisational parameters**
        """
        self.selection = selection

        """
        **Environmental parameters**
        """
        self.g = 9.81  # 9.80665

        """
        **Motor parameters**
        fl -> Motor Force Limit
        Rm -> Motor Armature Resistance (Ohm)
        Lm -> Motor Armature Inductance (H)
        Kt -> Motor Torque Constant (N.m/A)
        eta_m -> Motor Electromechanical Efficiency [ = Tm * w / ( Vm * Im ) ]
        Km -> Motor Back-EMF Constant (V.s/rad)
        Jm -> Rotor Inertia (kg.m^2)
        Mc -> IP02 Cart Mass
        Mw -> Cart Weight Mass (3 cable connectors) (kg)
        Kg -> Planetary Gearbox (a.k.a. Internal) Gear Ratio
        eta_g -> Planetary Gearbox Efficiency
        r_mp -> Motor Pinion Radius (m)
        Beq -> Equivalent Viscous Damping Coefficient as seen at the Motor Pinion (N.s/m)
        M -> Combined Weight of the Cart with Harness
        Jeq -> Lumped Mass of the Cart System (accounting for the rotor inertia)
        """
        self.fl = 5
        self.Rm = 2.6  # TRUE
        self.Lm = 1.8E-4  # TRUE
        self.Kt = 7.67E-3  # TRUE
        self.eta_m = 1  # 0.69
        self.Km = 7.67E-3  # TRUE
        self.Jm = 3.9E-7  # TRUE
        self.Mc = 0.57  # TRUE
        self.Mw = 0.37  # TRUE
        self.Kg = 3.71  # TRUE
        self.eta_g = 1
        self.r_mp = 6.35E-3  # TRUE
        self.Beq = 5.4  # TRUE
        self.M = self.Mc + self.Mw  # TRUE
        self.Jeq = self.M + self.eta_g * self.Kg ** 2 * self.Jm / self.r_mp ** 2

        """
        **Pendulum parameters**
        # Mp -> Pendulum Mass (with T-fitting)
        # Lp -> Pendulum Full Length (with T-fitting, from axis of rotation to tip)
        # lp -> Distance from Pivot to Centre Of Gravity
        # Jp -> Pendulum Moment of Inertia (kg.m^2) - approximation
        # Bp -> Equivalent Viscous Damping Coefficient (N.m.s/rad)
        """
        if selection == "short":
            self.Mp = 0.127
            self.Lp = 0.3365
            self.lp = 0.1778
            self.Jp = 1.1987E-3  # TRUE
            self.Bp = 0.0024
        elif selection == "long":
            self.Mp = 0.230
            self.Lp = 0.64135
            self.lp = 0.3302
            self.Jp = 3.344E-2
            # self.Jp = 7.8838E-3  # quanser
            self.Bp = 0.0024

        self.J_T = (self.Jeq + self.Mp) * self.Jp + self.Jeq * self.Mp * self.lp ** 2

    def statespace(self):
        A = 1 / self.J_T * np.array([[0, 0, self.J_T, 0], [0, 0, 0, self.J_T],
                                     [0, self.Mp ** 2 * self.lp ** 2 * self.g,
                                      -(self.Jp + self.Mp * self.lp ** 2) * self.Beq,
                                      -self.Mp * self.lp * self.Bp],
                                     [0, (self.Jeq + self.Mp) * self.Mp * self.lp * self.g,
                                      -self.Mp * self.lp * self.Beq,
                                      -(self.Jeq + self.Mp) * self.Bp]])
        B = 1 / self.J_T * np.array([[0], [0], [self.Jp + self.Mp * self.lp ** 2], [self.Mp * self.lp]])
        C = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        D = np.array([[0], [0]])

        # actuator dynamics
        A[2, 2] = A[2, 2] - B[2] * self.eta_g * self.Kg ** 2 * self.eta_m * self.Kt * self.Km / self.r_mp ** 2 / self.Rm
        A[3, 2] = A[3, 2] - B[3] * self.eta_g * self.Kg ** 2 * self.eta_m * self.Kt * self.Km / self.r_mp ** 2 / self.Rm
        B = self.eta_g * self.Kg * self.eta_m * self.Kt / self.r_mp / self.Rm * B
        return A, B, C, D

    # def amplitude(self, u, x_c_dot):
    #     V = self.Jeq * self.Rm * self.r_mp * u / (self.eta_g * self.Kg * self.eta_m * self.Kt) \
    #         + self.Kg * self.Km * x_c_dot / self.r_mp
    #     return V

    def amplitude(self, u, x_c_dot):
        V = u * self.Rm * self.r_mp / (self.eta_g * self.Kg * self.eta_m * self.Kt) \
            + self.Kg * self.Km * x_c_dot / self.r_mp
        return V

    def force(self, amplitude, x_c_dot):
        F = (self.eta_g * self.Kg * self.eta_m * self.Kt / (self.Rm * self.r_mp)) \
            * (-self.Kg * self.Km * x_c_dot / self.r_mp + amplitude)
        return F

    def continuous_dynamics(self, x, u):
        D = 4 * self.M * self.r_mp ** 2 + self.Mp * self.r_mp ** 2 + 4 * self.Jm * self.Kg ** 2 \
            + 3 * self.Mp * self.r_mp ** 2 * np.sin(x[1]) ** 2

        c_acc = - 3 * self.r_mp ** 2 * self.Bp * np.cos(x[1]) * x[3] / (self.lp * D) \
                - 4 * self.Mp * self.lp * self.r_mp ** 2 * np.sin(x[1]) * x[3] ** 2 / D \
                - 4 * self.r_mp ** 2 * self.Beq * x[2] / D \
                + 3 * self.Mp * self.r_mp ** 2 * self.g * np.cos(x[1]) * np.sin(x[1]) / D \
                + 4 * self.r_mp**2 * u[0] / (D * self.eta_g * self.eta_m)

        p_acc = - 3 * (self.M * self.r_mp**2 + self.Mp * self.r_mp**2 + self.Jm * self.Kg**2) * self.Bp * x[3] \
                / (self.Mp * self.lp**2 * D) - 3 * self.Mp * self.r_mp**2 * np.cos(x[1]) * np.sin(x[1]) * x[3]**2 \
                / D - 3 * self.r_mp**2 * self.Beq * np.cos(x[1]) * x[2] / (self.lp * D) \
                + 3 * (self.M * self.r_mp**2 + self.Mp * self.r_mp**2 + self.Jm * self.Kg**2) * self.g * np.sin(x[1]) \
                / (self.lp * D) + 3 * self.r_mp**2 * np.cos(x[1]) * u[0] / (self.lp * D * self.eta_g * self.eta_m)

        xd = np.array([x[2], x[3], c_acc, p_acc])

        dfdt,dfdx,dfdu,dfdw = self.partialDerivatives(x,u,0)
        dxd = [dfdt, dfdx, dfdu, dfdw]

        return xd, dxd
    
    def continuous_dynamics_RoA(self, x, u):
        q = np.array([x[0], x[1]])
        v = np.array([x[2], x[3]])
        M, Cv, tauG, B = self.getManipulatorDynamics(q,v)

        # M_inv = np.linalg.inv(M)
        det_M = M[0][0] * M[1][1] - M[0][1] * M[1][0]
        M_inv = (1/det_M)*np.array([[M[1][1], -M[1][0]], [-M[0][1], M[0][0]]])
        # v_dot = M_inv.dot(-Cv.dot(v.T) - tauG + B.dot(u))
        v_dot = M_inv.dot(-tauG + B.dot(u) - Cv)

        xd = np.array([v[0], v[1], v_dot[0], v_dot[1]])
        return xd

    def linearized_continuous_dynamics3(self, x, u, x_star, taylor_deg):
        f = self.continuous_dynamics_drake(x,u)
        c_acc = f[2]
        p_acc = f[3]

        env_lin = { x[0]: x_star[0],
                    x[1]: x_star[1],
                    x[2]: x_star[2],
                    x[3]: x_star[3]}
        c_acc_lin = sym.TaylorExpand(c_acc,env_lin,taylor_deg)
        p_acc_lin = sym.TaylorExpand(p_acc,env_lin,taylor_deg)

        xd = np.array([x[2], x[3], c_acc_lin, p_acc_lin])
        return xd
    
    def continuous_dynamics_w(self, x, u, w):
        x_w = np.array(x) + w

        D = 4 * self.M * self.r_mp ** 2 + self.Mp * self.r_mp ** 2 + 4 * self.Jm * self.Kg ** 2 \
            + 3 * self.Mp * self.r_mp ** 2 * np.sin(x_w[1]) ** 2

        c_acc = - 3 * self.r_mp ** 2 * self.Bp * np.cos(x_w[1]) * x_w[3] / (self.lp * D) \
                - 4 * self.Mp * self.lp * self.r_mp ** 2 * np.sin(x_w[1]) * x_w[3] ** 2 / D \
                - 4 * self.r_mp ** 2 * self.Beq * x_w[2] / D \
                + 3 * self.Mp * self.r_mp ** 2 * self.g * np.cos(x_w[1]) * np.sin(x_w[1]) / D \
                + 4 * self.r_mp**2 * u[0] / (D * self.eta_g * self.eta_m)

        p_acc = - 3 * (self.M * self.r_mp**2 + self.Mp * self.r_mp**2 + self.Jm * self.Kg**2) * self.Bp * x_w[3] \
                / (self.Mp * self.lp**2 * D) - 3 * self.Mp * self.r_mp**2 * np.cos(x_w[1]) * np.sin(x_w[1]) * x_w[3]**2 \
                / D - 3 * self.r_mp**2 * self.Beq * np.cos(x_w[1]) * x_w[2] / (self.lp * D) \
                + 3 * (self.M * self.r_mp**2 + self.Mp * self.r_mp**2 + self.Jm * self.Kg**2) * self.g * np.sin(x_w[1]) \
                / (self.lp * D) + 3 * self.r_mp**2 * np.cos(x_w[1]) * u[0] / (self.lp * D * self.eta_g * self.eta_m)

        xd = np.array([x_w[2], x_w[3], c_acc, p_acc])

        dfdt,dfdx,dfdu,dfdw = self.partialDerivatives(x,u,w)
        dxd = [dfdt, dfdx, dfdu, dfdw]

        return xd, dxd

    def partialDerivatives(self, x, u, w):  
        df0dx0=  0
        df1dx0=  0
        df0dx1=  0
        df1dx1=  0
        df0dx2=  1
        df1dx2=  0
        df0dx3=  0
        df1dx3=  1
        df0du =  0
        df1du =  0
        df0dw =  1
        df1dw =  1
        df2dx0=  0
        df3dx0=  0
        df2dx1=  24*self.Beq*self.Mp*self.r_mp**4*(w + x[2])*sin(w + x[1])*cos(w + x[1])/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2 + 18*self.Bp*self.Mp*self.r_mp**4*(w + x[3])*sin(w + x[1])*cos(w + x[1])**2/(self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2) + 3*self.Bp*self.r_mp**2*(w + x[3])*sin(w + x[1])/(self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)) - 18*self.Mp**2*self.g*self.r_mp**4*sin(w + x[1])**2*cos(w + x[1])**2/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2 + 24*self.Mp**2*self.lp*self.r_mp**4*(w + x[3])**2*sin(w + x[1])**2*cos(w + x[1])/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2 - 3*self.Mp*self.g*self.r_mp**2*sin(w + x[1])**2/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2) + 3*self.Mp*self.g*self.r_mp**2*cos(w + x[1])**2/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2) - 4*self.Mp*self.lp*self.r_mp**2*(w + x[3])**2*cos(w + x[1])/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2) - 24*self.Mp*self.r_mp**4*u[0]*sin(w + x[1])*cos(w + x[1])/(self.eta_g*self.eta_m*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2)
        df3dx1=  18*self.Beq*self.Mp*self.r_mp**4*(w + x[2])*sin(w + x[1])*cos(w + x[1])**2/(self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2) + 3*self.Beq*self.r_mp**2*(w + x[2])*sin(w + x[1])/(self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)) - 6*self.Bp*self.r_mp**2*(w + x[3])*(-3*self.Jm*self.Kg**2 - 3*self.M*self.r_mp**2 - 3*self.Mp*self.r_mp**2)*sin(w + x[1])*cos(w + x[1])/(self.lp**2*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2) + 18*self.Mp**2*self.r_mp**4*(w + x[3])**2*sin(w + x[1])**2*cos(w + x[1])**2/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2 - 6*self.Mp*self.g*self.r_mp**2*(3*self.Jm*self.Kg**2 + 3*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2)*sin(w + x[1])**2*cos(w + x[1])/(self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2) + 3*self.Mp*self.r_mp**2*(w + x[3])**2*sin(w + x[1])**2/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2) - 3*self.Mp*self.r_mp**2*(w + x[3])**2*cos(w + x[1])**2/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2) - 18*self.Mp*self.r_mp**4*u[0]*sin(w + x[1])*cos(w + x[1])**2/(self.eta_g*self.eta_m*self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2) + self.g*(3*self.Jm*self.Kg**2 + 3*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2)*cos(w + x[1])/(self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)) - 3*self.r_mp**2*u[0]*sin(w + x[1])/(self.eta_g*self.eta_m*self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2))
        df2dx2=  -4*self.Beq*self.r_mp**2/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)
        df3dx2=  -3*self.Beq*self.r_mp**2*cos(w + x[1])/(self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2))
        df2dx3=  -3*self.Bp*self.r_mp**2*cos(w + x[1])/(self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)) - 4*self.Mp*self.lp*self.r_mp**2*(2*w + 2*x[3])*sin(w + x[1])/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)
        df3dx3=  self.Bp*(-3*self.Jm*self.Kg**2 - 3*self.M*self.r_mp**2 - 3*self.Mp*self.r_mp**2)/(self.Mp*self.lp**2*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)) - 3*self.Mp*self.r_mp**2*(2*w + 2*x[3])*sin(w + x[1])*cos(w + x[1])/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)
        df2du =  4*self.r_mp**2/(self.eta_g*self.eta_m*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2))
        df3du =  3*self.r_mp**2*cos(w + x[1])/(self.eta_g*self.eta_m*self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2))
        df2dw =  24*self.Beq*self.Mp*self.r_mp**4*(w + x[2])*sin(w + x[1])*cos(w + x[1])/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2 - 4*self.Beq*self.r_mp**2/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2) + 18*self.Bp*self.Mp*self.r_mp**4*(w + x[3])*sin(w + x[1])*cos(w + x[1])**2/(self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2) + 3*self.Bp*self.r_mp**2*(w + x[3])*sin(w + x[1])/(self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)) - 3*self.Bp*self.r_mp**2*cos(w + x[1])/(self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)) - 18*self.Mp**2*self.g*self.r_mp**4*sin(w + x[1])**2*cos(w + x[1])**2/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2 + 24*self.Mp**2*self.lp*self.r_mp**4*(w + x[3])**2*sin(w + x[1])**2*cos(w + x[1])/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2 - 3*self.Mp*self.g*self.r_mp**2*sin(w + x[1])**2/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2) + 3*self.Mp*self.g*self.r_mp**2*cos(w + x[1])**2/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2) - 4*self.Mp*self.lp*self.r_mp**2*(w + x[3])**2*cos(w + x[1])/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2) - 4*self.Mp*self.lp*self.r_mp**2*(2*w + 2*x[3])*sin(w + x[1])/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2) - 24*self.Mp*self.r_mp**4*u[0]*sin(w + x[1])*cos(w + x[1])/(self.eta_g*self.eta_m*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2)
        df3dw =  18*self.Beq*self.Mp*self.r_mp**4*(w + x[2])*sin(w + x[1])*cos(w + x[1])**2/(self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2) + 3*self.Beq*self.r_mp**2*(w + x[2])*sin(w + x[1])/(self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)) - 3*self.Beq*self.r_mp**2*cos(w + x[1])/(self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)) - 6*self.Bp*self.r_mp**2*(w + x[3])*(-3*self.Jm*self.Kg**2 - 3*self.M*self.r_mp**2 - 3*self.Mp*self.r_mp**2)*sin(w + x[1])*cos(w + x[1])/(self.lp**2*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2) + self.Bp*(-3*self.Jm*self.Kg**2 - 3*self.M*self.r_mp**2 - 3*self.Mp*self.r_mp**2)/(self.Mp*self.lp**2*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)) + 18*self.Mp**2*self.r_mp**4*(w + x[3])**2*sin(w + x[1])**2*cos(w + x[1])**2/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2 - 6*self.Mp*self.g*self.r_mp**2*(3*self.Jm*self.Kg**2 + 3*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2)*sin(w + x[1])**2*cos(w + x[1])/(self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2) + 3*self.Mp*self.r_mp**2*(w + x[3])**2*sin(w + x[1])**2/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2) - 3*self.Mp*self.r_mp**2*(w + x[3])**2*cos(w + x[1])**2/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2) - 3*self.Mp*self.r_mp**2*(2*w + 2*x[3])*sin(w + x[1])*cos(w + x[1])/(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2) - 18*self.Mp*self.r_mp**4*u[0]*sin(w + x[1])*cos(w + x[1])**2/(self.eta_g*self.eta_m*self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)**2) + self.g*(3*self.Jm*self.Kg**2 + 3*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2)*cos(w + x[1])/(self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2)) - 3*self.r_mp**2*u[0]*sin(w + x[1])/(self.eta_g*self.eta_m*self.lp*(4*self.Jm*self.Kg**2 + 4*self.M*self.r_mp**2 + 3*self.Mp*self.r_mp**2*sin(w + x[1])**2 + self.Mp*self.r_mp**2))

        dfdx = [[df0dx0, df0dx1, df0dx2, df0dx3], 
                [df1dx0, df1dx1, df1dx2, df1dx3],
                [df2dx0, df2dx1, df2dx2, df2dx3],
                [df3dx0, df3dx1, df3dx2, df3dx3]]
        dfdu = [df0du, df1du, df2du, df3du]
        dfdw = [df0dw, df1dw, df2dw, df3dw]
        dfdt = np.zeros((4,1))

        return dfdt,dfdx,dfdu,dfdw

    def continuous_dynamics_drake(self,x,u):
        D = 4 * self.M * self.r_mp ** 2 + self.Mp * self.r_mp ** 2 + 4 * self.Jm * self.Kg ** 2 \
            + 3 * self.Mp * self.r_mp ** 2 * sym.sin(x[1]) ** 2

        c_acc = - 3 * self.r_mp ** 2 * self.Bp * sym.cos(x[1]) * x[3] / (self.lp * D) \
                - 4 * self.Mp * self.lp * self.r_mp ** 2 * sym.sin(x[1]) * x[3] ** 2 / D \
                - 4 * self.r_mp ** 2 * self.Beq * x[2] / D \
                + 3 * self.Mp * self.r_mp ** 2 * self.g * sym.cos(x[1]) * sym.sin(x[1]) / D \
                + 4 * self.r_mp**2 * u[0] / (D * self.eta_g * self.eta_m)

        p_acc = - 3 * (self.M * self.r_mp**2 + self.Mp * self.r_mp**2 + self.Jm * self.Kg**2) * self.Bp * x[3] \
                / (self.Mp * self.lp**2 * D) - 3 * self.Mp * self.r_mp**2 * sym.cos(x[1]) * sym.sin(x[1]) * x[3]**2 \
                / D - 3 * self.r_mp**2 * self.Beq * sym.cos(x[1]) * x[2] / (self.lp * D) \
                + 3 * (self.M * self.r_mp**2 + self.Mp * self.r_mp**2 + self.Jm * self.Kg**2) * self.g * sym.sin(x[1]) \
                / (self.lp * D) + 3 * self.r_mp**2 * sym.cos(x[1]) * u[0] / (self.lp * D * self.eta_g * self.eta_m)
        xd = np.array([x[2], x[3], c_acc, p_acc])
        return xd

def generateUrdf(Mp,lp,Jp, name = None): 
    # Compile templates into URDF robot description
    loader = FileSystemLoader(searchpath="data/cart_pole/urdfs/template")
    env = Environment(loader=loader, autoescape=True)

    dir = "data/cart_pole/urdfs/"
    if name is None:
        t = dir + "cartpole_CMAES.urdf"
    else:
        t = dir + name
    template = env.get_template("cartpoleTemplate.j2")
    f = open(t, "w")
    f.write(template.render(Mp = Mp, 
                            lp = lp,
                            Jp = Jp))
    
    return t