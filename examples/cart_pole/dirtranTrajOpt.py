import time
import numpy as np
import matplotlib as mpl
mpl.use("WebAgg")
import matplotlib.pyplot as plt

from cart_pole.trajectory_optimization.dirtran.dirtranTrajOpt import DirtranTrajectoryOptimization
from cart_pole.model.parameters import Cartpole
from cart_pole.model.parameters import generateUrdf

wantToSave = True
save_dir = "data/cart_pole/dirtran/trajectory.csv"

# Cart-pole system init
sys = Cartpole("short")
old_Mp = sys.Mp
sys.Mp = 0.227
sys.Jp = sys.Jp + (sys.Mp-old_Mp)*(sys.lp**2)
sys.fl = 6
urdf_path = generateUrdf(sys.Mp, sys.lp, sys.Jp)

# Direct transcription parameters
options = {"N": 201,
           "x0": [0, np.pi, 0, 0],
           "xG": [0, 0, 0, 0],
           "hBounds": [0.01, 0.06],
           "fl": sys.fl,
           "cart_pos_lim": 0.3,
           "QN": np.diag([100, 100, 100, 100]),
           "R": 10,
           "Q":  np.diag([10, 10, 1, 1]),
           "time_penalization": 0,
           "tf0": 8,
           "urdf": urdf_path }

# Direct transcription execution
dirtran = DirtranTrajectoryOptimization(sys, options)
t_start = time.time()
print(f'Starting Day Time: {time.strftime("%H:%M:%S", time.localtime())}')
[T, X, U] = dirtran.ComputeTrajectory()
print(f'Calculation Time(sec): {(time.time() - t_start)}')
T = np.array(T)[:, np.newaxis]
U = np.array(U)[:, np.newaxis]
X = X.T

# Trajectory saving
if wantToSave:
    traj_data = np.hstack((T, X, U))
    print(traj_data.shape)
    np.savetxt(save_dir, traj_data, delimiter=',', header="time,cart_pos,pend_pos,cart_vel,pend_vel,force", comments="") 
    print("Trajectory saved in:", save_dir)

# Trajectory visualization
fig, ax = plt.subplots(5, 1, figsize=(18, 6), sharex="all")
ax[0].plot(T, X[:, 0] * 1000, label="x")
ax[0].set_ylabel("cart pos. [mm]")
ax[0].legend(loc="best")
ax[1].plot(T, X[:, 1], label="theta")
ax[1].set_ylabel("pend. pos. [rad]")
ax[1].legend(loc="best")
ax[2].plot(T, X[:, 2] * 1000, label="x_dot")
ax[2].set_ylabel("cart vel. [mm/s]")
ax[2].legend(loc="best")
ax[3].plot(T, X[:, 3], label="theta_dot")
ax[3].set_ylabel("pend. vel. [rad/s]")
ax[3].legend(loc="best")
ax[4].plot(T, U, label="u")
ax[4].set_xlabel("time [s]")
ax[4].set_ylabel("Force [N]")
ax[4].legend(loc="best")

plt.show()