import matplotlib as mpl
mpl.use("WebAgg")
mpl.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
})
import matplotlib.pyplot as plt
import numpy as np

from simple_pendulum.controllers.tvlqr.roa.plot import TVrhoVerification, plotFunnel3d
from simple_pendulum.utilities.process_data import prepare_trajectory
from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.controllers.tvlqr.tvlqr import TVLQRController

# Simulated verification parameters
optimized_traj_path = "results/simple_pendulum/optDesignCMAES_167329/trajectoryOptimal_CMAES.csv" 
optimized_funnel_path = "results/simple_pendulum/optDesignCMAES_167329/SosfunnelOptimal_CMAES.csv" 
optimal_pars_path = "results/simple_pendulum/optDesignCMAES_167329/fullCoopt_CMAES.csv" 
optimal_label = "RTCD"
verificationSamples = 100
verifiedKnot = 8

# Plot parameters
ticksSize = 40
fontSize = 40

#########################
# Simulated verification
#########################  

# Load optimal params
controller_data = np.loadtxt(optimal_pars_path, skiprows=1, delimiter=",")
max_idx = np.where(-controller_data.T[5] == max(-controller_data.T[5]))[0][0]
m_opt = controller_data[max_idx,0]
l_opt = controller_data[max_idx,1]
Q_opt = np.diag([controller_data[max_idx,2],controller_data[max_idx,3]]) 
R_opt = [controller_data[max_idx,4]]

print("Verifying RTC-D results...")
print("The optimal m is: ", m_opt)
print("The optimal l is: ", l_opt)
print("The optimal Q is: ", Q_opt)
print("The optimal R is: ", R_opt)

# Verification
optimized_data_dict = prepare_trajectory(optimized_traj_path)
pendulum_par = {"l": l_opt,
        "m": m_opt,
        "b": 0.1, 
        "g": 9.81,
        "cf": 0.0,
        "tl": 2.5}
pendulum_plant = PendulumPlant(mass=pendulum_par["m"],
                        length=pendulum_par["l"],
                        damping=pendulum_par["b"],
                        gravity=pendulum_par["g"],
                        coulomb_fric=pendulum_par["cf"],
                        inertia=None,
                        torque_limit=pendulum_par["tl"])
controller = TVLQRController(data_dict=optimized_data_dict, mass=pendulum_par["m"], length=pendulum_par["l"],
                            damping=pendulum_par["b"], gravity=pendulum_par["g"],
                            torque_limit=pendulum_par["tl"])
controller.set_costs(Q_opt, R_opt)
controller.set_goal([np.pi,0])
(ax_ellipse, ax_traj) = TVrhoVerification(pendulum_plant,controller,optimized_funnel_path, optimized_traj_path,verificationSamples,verifiedKnot, fontSize=fontSize, ticksSize=ticksSize)

################################
# Real experimental verification
################################

# load trajectories from csv file
trajDirtrandist_path = "results/simple_pendulum/realExperiments/167329dirtrandist/data_measured.csv"
trajRTCDdist_path = "results/simple_pendulum/realExperiments/167329rtcddist/data_measured.csv"
trajDirtrandist = np.loadtxt(trajDirtrandist_path, skiprows=1, delimiter=",")
dirtrandist_time_list = trajDirtrandist.T[0].T  
dirtrandist_pos_list = trajDirtrandist.T[1].T  
dirtrandist_vel_list = trajDirtrandist.T[2].T  
dirtrandist_tau_list = trajDirtrandist.T[3].T   
trajOptRTCDdist = np.loadtxt(trajRTCDdist_path, skiprows=1, delimiter=",")
rtcddist_time_list = trajOptRTCDdist.T[0].T  
rtcddist_pos_list  = trajOptRTCDdist.T[1].T  
rtcddist_vel_list  = trajOptRTCDdist.T[2].T  
rtcddist_tau_list  = trajOptRTCDdist.T[3].T

# Plotting the real traj in the funnel
ax_traj, nominal = plotFunnel3d(optimized_funnel_path, optimized_traj_path,fontSize=fontSize,ticksSize=ticksSize)
dirtran, =ax_traj.plot(dirtrandist_time_list[0:], dirtrandist_pos_list[0:], dirtrandist_vel_list[0:], label = "DIRTRAN", color = "C3", linewidth = "0.7")
opt2, = ax_traj.plot(rtcddist_time_list[0:], rtcddist_pos_list[0:], rtcddist_vel_list[0:], label = "RTC-D", color = "C2", linewidth = "0.7")
ax_traj.legend(handles = [nominal,dirtran, opt2], fontsize = fontSize)

plt.show()