import numpy as np
import os
from pathlib import Path
import sympy as smp
from ilqr.pendulum import (discrete_dynamics_euler, discrete_dynamics_rungekutta, swingup_stage_cost, swingup_final_cost)
from model.parameters import Cartpole
from ilqr.pendulum import discrete_dynamics_euler
from controllers.lqr.lqr import lqr
import matplotlib.pyplot as plt

sys = Cartpole("short")
x_c_dot = 0.12542
force_ini = 5.82372
amplitude = sys.amplitude(force_ini, x_c_dot)
force = sys.force(amplitude, x_c_dot)
print(f'Force: {force}')
if force == force_ini:
    print(f'Hurray')

N = 1000
dt = 0.01
x0 = np.array([0, np.pi, 0, 0])
t = np.linspace(0, N-1, N)*dt
u_trj = np.array([-1 * np.sin(2 * np.pi * t / 2)]).T
# u_trj[750:] = 0
# u_trj = np.ones(t.shape)[:, np.newaxis] * 2

# from swingup
# csv_path = os.path.join("log_data", "trajectory_swingup.csv")
# trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
# u_trj = trajectory.T[5].T[:, np.newaxis]  # input array
print(f'{u_trj}')
# N = u_trj.shape[0]
# dt = trajectory.T[0].T[1]

x_trj = np.zeros((u_trj.shape[0]+1, x0.shape[0]))
x = x0
i = 0
x_trj[i, :] = x
k_loop = 0
for u in u_trj:
    i = i+1
    k_loop += 1
    x = discrete_dynamics_euler(sys, x, u, dt)
    # x = discrete_dynamics_rungekutta(sys, x, u, dt)
    x_trj[i, :] = x

# x_trj[-1, 1:] = [np.pi, 0, 0]
# time = np.linspace(0, N, N+1) * dt
CART_POS = x_trj.T[0]
PEND_POS = x_trj.T[1]
CART_VEL = x_trj.T[2]
PEND_VEL = x_trj.T[3]

plt.figure(figsize=(12, 6))
plt.subplot(5, 1, 1)
plt.grid(True)
plt.plot(t, x_trj.T[0, :-1] * 1000, label='des')
plt.ylabel('Cart Pos [mm]')
plt.legend()
plt.subplot(5, 1, 2)
plt.grid(True)
plt.plot(t, x_trj.T[1, :-1], label='des')
plt.ylabel('Pend Pos [rad]')
plt.legend()
plt.subplot(5, 1, 3)
plt.grid(True)
plt.plot(t, x_trj.T[2, :-1] * 1000, label='des')
plt.ylabel('Cart vel [mm/s]')
plt.legend()
plt.subplot(5, 1, 4)
plt.grid(True)
plt.plot(t, x_trj.T[3, :-1], label='des')
plt.ylabel('Pend vel [rad/s]')
plt.legend()
plt.subplot(5, 1, 5)
plt.grid(True)
plt.plot(t, u_trj, label='des')
plt.ylabel('Force [N]')
plt.legend()
plt.show()

# plt.figure(figsize=(12, 6))
# plt.subplot(5, 1, 1)
# plt.plot(t[:k_loop], trajectory.T[1].T * 1000, label='des')
# plt.plot(t[:k_loop], state[0, :k_loop] * 1000, label='mea')
# plt.ylabel('Cart Position [mm]')
# plt.legend()
# plt.subplot(5, 1, 2)
# pendulum_position_desired = (trajectory.T[2].T + np.pi) % (2 * np.pi) - np.pi
# pendulum_position_measured = (state[1, :k_loop] + np.pi) % (2 * np.pi) - np.pi
# plt.plot(t[:k_loop], pendulum_position_desired, label='des')
# plt.plot(t[:k_loop], pendulum_position_measured, label='mea')
# plt.ylabel('Pendulum Position [rad]')
# plt.xlabel('Time [sec]')
# plt.legend()
# plt.subplot(5, 1, 3)
# plt.plot(t[:k_loop], trajectory.T[3].T * 1000, label='des')
# plt.plot(t[:k_loop], state[2, :k_loop] * 1000, label='mea')
# plt.ylabel('Cart Velocity [mm/s]')
# plt.xlabel('Time [sec]')
# plt.legend()
# plt.subplot(5, 1, 4)
# plt.plot(t[:k_loop], trajectory.T[4].T, label='des')
# plt.plot(t[:k_loop], state[3, :k_loop], label='mea')
# plt.ylabel('Pendulum Velocity [rad/s]')
# plt.xlabel('Time [sec]')
# plt.legend()
# plt.subplot(5, 1, 5)
# plt.plot(t[:k_loop], trajectory.T[5].T, label='des')
# plt.plot(t[:k_loop], force[:k_loop], label='mea')
# plt.ylabel('u | Cart Force [N]')
# plt.xlabel('Time [sec]')
# plt.legend()
# plt.show()

time = np.linspace(0, N, N+1)*dt
WORK_DIR = Path(Path(os.path.abspath(__file__)).parents[1])
print("Workspace is set to:", WORK_DIR)
csv_file = "trajectory_sin.csv"
csv_path = os.path.join(WORK_DIR, 'data', 'trajectories', csv_file)
csv_data = np.vstack((time, CART_POS, PEND_POS, CART_VEL, PEND_VEL, np.append(u_trj.T[0], 0.0))).T
np.savetxt(csv_path, csv_data, delimiter=',', header="time,cart_pos,pend_pos,cart_vel,pend_vel,force", comments="")
