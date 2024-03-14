import numpy as np
import matplotlib as mlp
import tikzplotlib
import matplotlib.pyplot as plt
import time

from simple_pendulum.controllers.tvlqr.roa.utils import funnelVolume_convexHull, funnelVolume
from simple_pendulum.controllers.tvlqr.roa.plot import plotFunnel, rhoComparison, funnel2DComparison
from simple_pendulum.utilities.process_data import prepare_trajectory, saveFunnel

traj_path1 = "results/simple_pendulum/dirtran/trajectory.csv"
funnel_path1 = "results/simple_pendulum/dirtran/Sosfunnel.csv"
label1 = "DIRTRAN"
traj_path2 = "results/simple_pendulum/optDesignCMAES_167329/trajectoryOptimal_CMAES.csv"
funnel_path2 = "results/simple_pendulum/optDesignCMAES_167329/SosfunnelOptimal_CMAES.csv"
label2 = "RTCD"
traj_path3 = "results/simple_pendulum/optCMAES_167329/trajectoryOptimal_CMAES.csv" 
funnel_path3 = "results/simple_pendulum/optCMAES_167329/SosfunnelOptimal_CMAES.csv"
label3 = "RTC"

# Plots parameters
ticksSize = 40
fontSize = 40

#########################
# Trajectories comparison
#########################

# Load trajectories
trajectory1 = np.loadtxt(traj_path1, skiprows=1, delimiter=",")
T1 = trajectory1.T[0].T
X1 = [trajectory1.T[1].T, trajectory1.T[2].T]
U1 = trajectory1.T[3].T
trajectory2 = np.loadtxt(traj_path2, skiprows=1, delimiter=",")
T2 = trajectory2.T[0].T
X2 = [trajectory2.T[1].T, trajectory2.T[2].T]
U2 = trajectory2.T[3].T

# Comparison plots
fig, axs = plt.subplots(3,1, figsize=(18, 9))
axs[0].plot(T1,X1[0], label = label1)
axs[0].plot(T2,X2[0], label = label2)
axs[0].legend(loc = "lower right", fontsize = fontSize)
axs[0].set_ylabel(r'$\theta$'+" [rad]", fontsize = fontSize)
axs[0].grid(True)
axs[1].tick_params(axis='both', which='major', labelsize=ticksSize)
axs[1].plot(T1,X1[1], label = label1)
axs[1].plot(T2,X2[1], label = label2)
axs[1].legend(loc = "lower right", fontsize = fontSize)
axs[1].set_ylabel(r'$\dot \theta$'+" [rad]", fontsize = fontSize)
axs[1].grid(True)
axs[1].tick_params(axis='both', which='major', labelsize=ticksSize)
axs[2].plot(T1,U1, label = label1)
axs[2].plot(T2,U2, label = label2)
axs[2].legend(loc = "upper right", fontsize = fontSize)
axs[2].set_ylabel(r'$u$'+" [Nm]", fontsize = fontSize)
axs[2].set_xlabel("time [s]", fontsize = fontSize)
axs[2].grid(True)
axs[2].tick_params(axis='both', which='major', labelsize=ticksSize)

####################
# Funnels comparison
####################

# Volume computation
vol1 = funnelVolume_convexHull(funnel_path1, traj_path1)
vol2 = funnelVolume_convexHull(funnel_path2, traj_path2)
vol3 = funnelVolume_convexHull(funnel_path3, traj_path3)
print("The convex hull volume of the "+ label1 +" funnel is", vol1)
print("The convex hull volume of the "+ label2 +" funnel is", vol2)
print("The convex hull volume of the "+ label3 +" funnel is", vol3)

# Funnel comparison RTCD and RTC
ax0 = plotFunnel(funnel_path2, traj_path2, fontSize= fontSize, ticksSize= ticksSize, noTraj = True, funnel_color='green')
plotFunnel(funnel_path3, traj_path3, ax=ax0, fontSize= fontSize, ticksSize= ticksSize, noTraj = True, funnel_color='orange')
plotFunnel(funnel_path1, traj_path1, ax=ax0, fontSize= fontSize, ticksSize= ticksSize, noTraj = True)
g_patch = mlp.patches.Patch(color='green', label='RTC-D')
lg_patch = mlp.patches.Patch(color='orange', label='RTC')
r_patch = mlp.patches.Patch(color='red', label='Initial')
leg = ax0.legend(handles=[r_patch,lg_patch, g_patch], fontsize=fontSize,loc = "upper right")

##############################
# Optimization evolution plots
##############################

fullcoopt_path = "results/simple_pendulum/optDesignCMAES_167329/fullCoopt_CMAES.csv" 
inner_coopt_path = "results/simple_pendulum/optDesignCMAES_167329/CooptData_CMAES.csv"
coopt_path = "results/simple_pendulum/optCMAES_167329/CooptData_CMAES.csv"  
evolution_path = "results/simple_pendulum/optCMAES_167329/outcmaes/evolution.csv"
fullcoopt_data = np.loadtxt(fullcoopt_path, delimiter=",")
coopt_data = np.loadtxt(coopt_path, delimiter=",")
inner_coopt_data = np.loadtxt(inner_coopt_path, delimiter=",")
evolution_data = np.loadtxt(evolution_path, delimiter=",")
V = np.sort(-np.array(coopt_data.T[-1]))
V_full = np.sort(-np.array(fullcoopt_data.T[-1]))
m_var_full = []
l_var_full = []
r_var_full = []
ml_var_full = []
for i in range(len(V_full)):
    idx = np.where(-np.array(fullcoopt_data.T[-1]) == V_full[i])
    m_var_full = np.append(m_var_full, fullcoopt_data.T[0][idx])
    l_var_full = np.append(l_var_full, fullcoopt_data.T[1][idx])
    r_var_full = np.append(r_var_full, fullcoopt_data.T[4][idx])
    ml_var_full = np.append(ml_var_full, fullcoopt_data.T[0][idx][0]*fullcoopt_data.T[1][idx][0])
r_var = []
q11_var = []
q22_var = []
for i in range(len(V)):
    idx = np.where(-np.array(coopt_data.T[-1]) == V[i])
    r_var = np.append(r_var, coopt_data.T[2][idx])
    q11_var = np.append(q11_var, coopt_data.T[0][idx])
    q22_var = np.append(q22_var, coopt_data.T[1][idx])

cost = []
V_inner = np.sort(-np.array(inner_coopt_data.T[-1]))
for i in range(len(V_full)):
    if V_full[i] > 0.1:
        for j in range(int(len(V_inner)/len(V_full))):
            cost = np.append(cost, V_full[i])
cost = np.append([15.75], cost)
eval_full = np.array([i for i in range(len(cost))])
costRTC = []
iterations = np.array(evolution_data.T[-1])
V_ev = np.sort(-np.array(evolution_data.T[3]))
for i in range(len(V_ev)):
    if i == 0:
        j_prev = 0
    else:
        j_prev = int(iterations[i-1])
    for j in range(j_prev, int(iterations[i])):
        costRTC = np.append(costRTC, V_ev[i])
costRTC = np.append([cost[0]], costRTC)
eval = np.array([i for i in range(len(costRTC))])

# ROA volume evolution plot
plt.figure(figsize=(12,9))
plt.plot(eval_full,cost, label = "RTCD", color = "C1")
plt.plot(eval,costRTC, label = "RTC", color = "C0")
plt.grid(True)
plt.xlabel("Evaluations", fontsize=fontSize)
plt.ylabel("ROA Volume", fontsize=fontSize)
plt.xticks(fontsize=ticksSize)
plt.yticks(fontsize=ticksSize)
plt.rc('legend', fontsize=fontSize)
plt.legend(loc = "upper left")

# Parameters evolution plot
plt.figure(figsize=(11,9))
plt.scatter(V_full, np.array(ml_var_full)*9.81, label = r"$mgl_{RTCD}\ [Nm]$", color = "C1", marker = ".",s=150)
plt.hlines(2.5,0,V_full.max(),colors=["C1"], linestyles="--")
plt.text(60,2.52,r"$\tau_{lim}$",color = "C1", fontsize=fontSize)
plt.scatter(V, r_var, label = r"$r_{RTC}$", color = "C0", marker = ".", s=150)
plt.grid(True)
plt.xlabel("ROA Volume", fontsize=fontSize)
plt.ylabel("Optimization Parameters", fontsize=fontSize)
plt.xticks(fontsize=ticksSize)
plt.yticks(fontsize=ticksSize)
plt.xlim(0,V_full.max())
plt.ylim(-1,8)
plt.rc('legend', fontsize=fontSize)
plt.legend(loc = "upper left") 

plt.show()