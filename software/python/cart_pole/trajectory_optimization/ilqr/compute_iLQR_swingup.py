import os
import time
import numpy as np
import matplotlib.pyplot as plt

from model.parameters import Cartpole
from ilqr_sympy import iLQR_Calculator
from pendulum import (discrete_dynamics_euler, discrete_dynamics_rungekutta, swingup_stage_cost, swingup_final_cost)

i = np.sin(np.pi/2)
print(f'{i}')

log_dir = "log_data"
sys = Cartpole('short')

# pendulum parameters
integrator = "euler"
n_x = 4
n_u = 1

# swing-up parameters
x0 = np.array([0.0, np.pi, 0.0, 0.0])
dt = 0.02
goal = np.array([0.0, 0.0, 0.0, 0.0])

# ilqr parameters
N = 600
max_iter = 100
regu_init = 100

iLQR = iLQR_Calculator(sys, n_x, n_u, dt, goal)

# set dynamics
if integrator == "euler":
    dyn_func = discrete_dynamics_euler
else:
    dyn_func = discrete_dynamics_rungekutta

iLQR.set_discrete_dynamics(dyn_func)

s_cost_func = swingup_stage_cost
f_cost_func = swingup_final_cost
iLQR.set_stage_cost(s_cost_func)
iLQR.set_final_cost(f_cost_func)

iLQR.init_derivatives()
iLQR.set_start(x0)

# computation
t_start = time.time()
(x_trj, u_trj, cost_trace, regu_trace,
 redu_ratio_trace, redu_trace) = iLQR.run_ilqr(N=N,
                                               init_u_trj=None,
                                               init_x_trj=None,
                                               max_iter=max_iter,
                                               regu_init=regu_init,
                                               break_cost_redu=1e-6)
print(f'Calculation time: {np.round(time.time() - t_start)}s')

# save results
time = np.linspace(0, N-1, N)*dt
CART_POS = x_trj.T[0]
PEND_POS = x_trj.T[1]
CART_VEL = x_trj.T[2]
PEND_VEL = x_trj.T[3]
csv_data = np.vstack((time, CART_POS, PEND_POS, CART_VEL, PEND_VEL,
                      np.append(u_trj.T[0], 0.0))).T

csv_path = os.path.join(log_dir, "trajectory.csv")
np.savetxt(csv_path, csv_data, delimiter=',',
           header="time,cart_pos,pend_pos,cart_vel,pend_vel,torque", comments="")

# plot results
fig, ax = plt.subplots(5, 1, figsize=(18, 6), sharex="all")

ax[0].plot(time, x_trj.T[0] * 1000, label="x")
ax[0].set_ylabel("cart pos. [mm]")
ax[0].legend(loc="best")
ax[1].plot(time, x_trj.T[1], label="theta")
ax[1].set_ylabel("pend. pos. [rad]")
ax[1].legend(loc="best")
ax[2].plot(time, x_trj.T[2] * 1000, label="x_dot")
ax[2].set_ylabel("cart vel. [mm/s]")
ax[2].legend(loc="best")
ax[3].plot(time, x_trj.T[3], label="theta_dot")
ax[3].set_ylabel("pend. vel. [rad/s]")
ax[3].legend(loc="best")
# ax[4].plot(time, x_trj.T[3], label="theta_dot")
ax[4].plot(dt*np.linspace(0, N, np.shape(u_trj)[0]), u_trj, label="u")
ax[4].set_xlabel("time [s]")
ax[4].set_ylabel("Force [N]")
ax[4].legend(loc="best")
plt.show()

plt.subplots(figsize=(10, 6))
plt.subplot(2, 2, 1)
plt.plot(cost_trace)
plt.xlabel('# Iteration')
plt.ylabel('Total cost')
plt.title('Cost trace')

plt.subplot(2, 2, 2)
delta_opt = (np.array(cost_trace) - cost_trace[-1])
plt.plot(delta_opt)
plt.yscale('log')
plt.xlabel('# Iteration')
plt.ylabel('Optimality gap')
plt.title('Convergence plot')

plt.subplot(2, 2, 3)
plt.plot(redu_ratio_trace)
plt.title('Ratio of actual reduction and expected reduction')
plt.ylabel('Reduction ratio')
plt.xlabel('# Iteration')

plt.subplot(2, 2, 4)
plt.plot(regu_trace)
plt.title('Regularization trace')
plt.ylabel('Regularization')
plt.xlabel('# Iteration')
plt.tight_layout()

plt.show()
