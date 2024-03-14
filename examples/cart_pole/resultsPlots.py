import numpy as np
import matplotlib as mlp
mlp.use("WebAgg")
import matplotlib.pyplot as plt

from cart_pole.controllers.tvlqr.RoAest.plot import plotFunnel
from cart_pole.controllers.tvlqr.RoAest.utils import funnelVolume_convexHull

traj_path1 = "results/cart_pole/dirtran/trajectory.csv"
funnel_path1 = "results/cart_pole/optCMAES_31082023-11:59:46_volumeDIRTRAN/initRoA_CMAES.csv"
label1 = "DIRTRAN"
traj_path2 = "results/cart_pole/optCMAES_31082023-11:59:46_volumeDIRTRAN/trajectoryOptimal_CMAES.csv"
funnel_path2 = "results/cart_pole/optCMAES_31082023-11:59:46_volumeDIRTRAN/RoA_CMAES.csv"
label2 = "RTC"

# Plots parameters
ticksSize = 40
fontSize = 40
labels = [r"$x_{cart}$ [m]",r"$\theta$ [rad]",r"$\dot x_{cart}$ [m/s]",r"$\dot \theta$ [rad/s]", r"F [N]"]
indexes = (1,3) # Meaningful values (0,1) (0,2) (0,3) (1,2) (1,3) (2,3)

#########################
# Trajectories comparison
#########################

# Load trajectories
trajectory1 = np.loadtxt(traj_path1, skiprows=1, delimiter=",")
T1 = trajectory1.T[0].T
X1 = [trajectory1.T[1].T, trajectory1.T[2].T,trajectory1.T[3].T, trajectory1.T[4].T]
U1 = trajectory1.T[3].T
trajectory2 = np.loadtxt(traj_path2, skiprows=1, delimiter=",")
T2 = trajectory2.T[0].T
X2 = [trajectory2.T[1].T, trajectory2.T[2].T,trajectory2.T[3].T, trajectory2.T[4].T]
U2 = trajectory2.T[3].T

# Comparison plots
fig, ax = plt.subplots(5, 1, figsize=(16, 10), sharex="all")
ax[0].plot(T1, X1[0], label=r"DIRTRAN")
ax[0].plot(T2, X2[0], label=r"RTC")
ax[0].set_ylabel(labels[0], fontsize = fontSize)
ax[0].legend(loc="upper right", fontsize = fontSize)
ax[1].plot(T1, X1[1])
ax[1].plot(T2, X2[1])
ax[1].set_ylabel(labels[1], fontsize = fontSize)
ax[2].plot(T1, X1[2])
ax[2].plot(T2, X2[2])
ax[2].set_ylabel(labels[2], fontsize = fontSize)
ax[3].plot(T1, X1[3])
ax[3].plot(T2, X2[3])
ax[3].set_ylabel(labels[3], fontsize = fontSize)
ax[4].plot(T1, U1)
ax[4].plot(T2, U2)
ax[4].set_xlabel("time [s]", fontsize = ticksSize)
ax[4].set_ylabel(labels[4], fontsize = fontSize)

####################
# Funnel comparison
####################

# Volume computation
vol1 = funnelVolume_convexHull(funnel_path1, traj_path1)
vol2 = funnelVolume_convexHull(funnel_path2, traj_path2)
print("The convex hull volume of the "+ label1 +" funnel is", vol1)
print("The convex hull volume of the "+ label2 +" funnel is", vol2)

# Funnel plot
ax = plotFunnel(funnel_path2, traj_path2, indexes, fontSize = fontSize, ticksSize = ticksSize)
plotFunnel(funnel_path1, traj_path1, indexes, ax, fontSize = fontSize, ticksSize = ticksSize)
g_patch = mlp.patches.Patch(color='green', label='RTC')
r_patch = mlp.patches.Patch(color='red', label='Initial')
ax.legend(handles=[r_patch, g_patch], fontsize=fontSize,loc = "upper right")

########################
# Optimal Cost Evolution
#######################

coopt_path = "results/cart_pole/optCMAES_31082023-11:59:46_volumeDIRTRAN/CooptData_CMAES.csv"   
coopt_data = np.loadtxt(coopt_path, delimiter=",")

# ROA volume evolution plot
V = np.sort(-np.array(coopt_data.T[-1]))
iterations = np.linspace(0,len(V)-1,len(V))
fig, ax = plt.subplots(1, 1, figsize=(9, 9))
ax.plot(iterations,V, label = "RTC")
ax.grid(True)
ax.set_xlabel("Evaluations", fontsize=ticksSize)
ax.set_ylabel("ROA Volume", fontsize=ticksSize)
ax.legend(loc = "upper left", fontsize=fontSize)

# Parameters evolution plot
r_var = []
q11_var = []
q22_var = []
for i in range(len(V)):
    idx = np.where(-np.array(coopt_data.T[-1]) == V[i])
    r_var = np.append(r_var, coopt_data.T[2][idx])
    q11_var = np.append(q11_var, coopt_data.T[0][idx])
    q22_var = np.append(q22_var, coopt_data.T[1][idx])
fig, ax = plt.subplots(1, 1, figsize=(9, 9))
ax.plot(iterations,r_var, label = "r RTC")
ax.scatter(iterations,q11_var, label = "q11 RTC")
ax.scatter(iterations,q22_var, label = "q22 RTC")
ax.grid(True)
ax.set_xlabel("Evaluations", fontsize=ticksSize)
ax.legend(loc = "upper left", fontsize=fontSize)

plt.show()