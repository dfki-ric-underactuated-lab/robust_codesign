import numpy as np
import matplotlib as mpl
mpl.use("WebAgg")
from matplotlib import patches
import matplotlib.pyplot as plt
import pandas

from cart_pole.model.parameters import Cartpole
from cart_pole.utilities.process_data import prepare_trajectory
from cart_pole.simulation.simulator import StepSimulator
from cart_pole.controllers.tvlqr.RoAest.utils import getEllipseFromCsv, sampleFromEllipsoid, quad_form
from cart_pole.controllers.tvlqr.RoAest.plot import plotFunnel3d
from cart_pole.model.parameters import generateUrdf

# Simulated verification parameters
traj_path1 = "results/cart_pole/dirtran/trajectory.csv"
funnel_path1 = "results/cart_pole/optCMAES_31082023-11:59:46_volumeDIRTRAN/initRoA_CMAES.csv"
label1 = "DIRTRAN"
traj_path2 = "results/cart_pole/optCMAES_31082023-11:59:46_volumeDIRTRAN/trajectoryOptimal_CMAES.csv"
funnel_path2 = "results/cart_pole/optCMAES_31082023-11:59:46_volumeDIRTRAN/RoA_CMAES.csv"
label2 = "RTC"
nVerifications = 100

# Plot parameters
ticksSize = 20
fontSize = 20
labels = [r"$x_{cart}$ [m]",r"$\theta$ [rad]",r"$\dot x_{cart}$ [m/s]",r"$\dot \theta$ [rad/s]", r"F [N]"]
indexes = (1,3) # Meaningful values (0,1) (0,2) (0,3) (1,2) (1,3) (2,3)

#########################
# Simulated verification
#########################

# Cart-pole system init
sys = Cartpole("short")
old_Mp = sys.Mp
sys.Mp = 0.227
sys.Jp = sys.Jp + (sys.Mp-old_Mp)*(sys.lp**2)
sys.fl = 6
urdf_path = generateUrdf(sys.Mp,sys.lp, sys.Jp)

# Load nominal trajectory
trajectory2 = np.loadtxt(traj_path2, skiprows=1, delimiter=",")
X2 = np.array([trajectory2.T[1], trajectory2.T[2], trajectory2.T[3], trajectory2.T[4]])
U2 = np.array([trajectory2.T[5]])
T2 = np.array([trajectory2.T[0]]).T 
traj_dict = prepare_trajectory(traj_path2)
traj_x1 = traj_dict["des_cart_pos_list"]
traj_x2 = traj_dict["des_pend_pos_list"]
traj_x3 = traj_dict["des_cart_vel_list"]
traj_x4 = traj_dict["des_pend_vel_list"]

# Simulator init
controller_options = {"T_nom": traj_dict["des_time_list"],
                        "U_nom": traj_dict["des_force_list"],
                        "X_nom": np.vstack((traj_x1, traj_x2, traj_x3, traj_x4)),
                        "Q": np.diag([10.91,12.57,1,1]),
                        "R": np.array([6.09]),
                        "xG": np.array([0,0,0,0])}
cartpole = {"urdf": urdf_path,
            "sys": sys,
            "x_lim": 0.3}
dt_sim = 0.008
sim = StepSimulator(cartpole, controller_options, verbose = False)

# Simulate grid sampled init states
ver_knot = 0
nSucc = 0
nFail = 0
onceFail = True
onceSucc = True
(rho0, S0) = getEllipseFromCsv(funnel_path2,ver_knot)
(rhoF, SF) = getEllipseFromCsv(funnel_path2,-1)
fig, ax = plt.subplots(2,2, figsize = (11, 9))
for i in range(nVerifications):
    xBar0 = sampleFromEllipsoid(S0, rho0)
    x0 = xBar0 + X2.T[ver_knot]
    if x0[0] > cartpole["x_lim"]:
        x0[0] = cartpole["x_lim"]
    elif x0[0] < -cartpole["x_lim"]:
        x0[0] = -cartpole["x_lim"]

    xBar0_clip = x0 - X2.T[ver_knot]
    if quad_form(S0, xBar0_clip) < rho0:
        sim.init_simulation(x0=x0,init_knot=ver_knot, dt_sim = dt_sim)
        T2_sim, X2_sim, U2_sim = sim.simulate()

        # Plot the results
        xBarF = X2_sim.T[-1] - X2.T[-1]
        if quad_form(SF, xBarF) < rhoF:
            trajColor = "green"
            nSucc += 1
        else:
            trajColor = "red"
            nFail += 1
        ax[0][0].plot(T2_sim, X2_sim[0], color = trajColor)
        if nSucc == 1 and onceSucc:
            ax[0][1].plot(T2_sim, X2_sim[1], color = trajColor, label ="Succeeding simulation")
            onceSucc = False
        elif nFail == 1 and onceFail:
            ax[0][1].plot(T2_sim, X2_sim[1], color = trajColor, label ="Failing simulation")
            onceFail = False
        else:
            ax[0][1].plot(T2_sim, X2_sim[1], color = trajColor)
        ax[1][0].plot(T2_sim, X2_sim[2], color = trajColor)
        ax[1][1].plot(T2_sim, X2_sim[3], color = trajColor)
        ax[0][0].hlines(np.vstack((np.ones((len(T2_sim),1)),-np.ones((len(T2_sim),1))))*0.3,T2_sim[0], T2_sim[-1])
        ax[0][0].set_ylabel(labels[0], fontsize = fontSize)
        ax[0][1].set_ylabel(labels[1], fontsize = fontSize)
        ax[1][0].set_ylabel(labels[2], fontsize = fontSize)
        ax[1][1].set_ylabel(labels[3], fontsize = fontSize)
        ax[1][0].set_xlabel("Time [s]", fontsize = fontSize)
        ax[1][1].set_xlabel("Time [s]", fontsize = fontSize)
        ax[0][1].legend(loc = "upper right", fontsize = fontSize)

ratio_perc = int((nSucc/(nSucc+nFail))*100)
print(f"The guarantee given by the simulation-based estimation method is not a formal guarantee but it still provides a satisfactory success ratio of {ratio_perc}%" )

################################
# Real experimental verification
################################

# Load real trajectories from csv file
trajDirtrandist_path = "results/cart_pole/realExperiments/167332dirtrandist.csv" 
trajDirtrandist = np.loadtxt(trajDirtrandist_path, skiprows=1, delimiter=",")
dirtrandist_time_list = trajDirtrandist.T[0].T  
dirtrandist_x0_list  = trajDirtrandist.T[indexes[0]+1].T  
dirtrandist_x1_list  = trajDirtrandist.T[indexes[1]+1].T 
dirtrandist_force_list = trajDirtrandist.T[5].T   
traj_pathRtc = "results/cart_pole/realExperiments/167332rtcdist.csv" 
trajOptRTC = np.loadtxt(traj_pathRtc, skiprows=1, delimiter=",")
rtc_time_list = trajOptRTC.T[0].T  
rtc_x0_list  = trajOptRTC.T[indexes[0]+1].T  
rtc_x1_list  = trajOptRTC.T[indexes[1]+1].T  
rtc_force_list  = trajOptRTC.T[5].T

# Plotting the real traj in the funnel
ax,nominal = plotFunnel3d(funnel_path2, traj_path2, indexes)
ax.plot(rtc_x0_list, rtc_x1_list, label = "RTC", color = "C2", linewidth = "0.7")
ax.plot(dirtrandist_x0_list, dirtrandist_x1_list, label = "DIRTRAN", color = "C3", linewidth = "0.7")
ax.legend(fontsize=fontSize,loc = "upper right")

plt.show()