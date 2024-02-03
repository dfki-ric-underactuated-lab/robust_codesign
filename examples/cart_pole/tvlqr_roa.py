import numpy as np
import numpy as np
import matplotlib as mpl
mpl.use("WebAgg")
import matplotlib.pyplot as plt
from time import time

from cart_pole.model.parameters import Cartpole
from cart_pole.utilities.process_data import prepare_trajectory
from cart_pole.simulation.simulator import StepSimulator
from cart_pole.controllers.tvlqr.RoAest.PROBest import probTVROA
from cart_pole.controllers.tvlqr.RoAest.utils import storeFunnel, funnelVolume_convexHull
from cart_pole.controllers.tvlqr.RoAest.plot import plotFunnel, TVfunnelVerification, plotRhoEvolution
from cart_pole.controllers.lqr.RoAest.SOSest import bisect_and_verify
from cart_pole.model.parameters import generateUrdf

from pydrake.all import Linearize, \
                        LinearQuadraticRegulator, \
                        DiagramBuilder, \
                        AddMultibodyPlantSceneGraph, \
                        Parser

# Cart-pole system init
sys = Cartpole("short")
old_Mp = sys.Mp
sys.Mp = 0.227
sys.Jp = sys.Jp + (sys.Mp-old_Mp)*(sys.lp**2)
sys.fl = 6
urdf_path = generateUrdf(sys.Mp,sys.lp, sys.Jp)

# Swing-up parameters
xG = np.array([0,0,0,0])

##################################
# RoA computation and Funnels plot
##################################

# Load trajectory computed fro dirtran
traj_path = "data/cart_pole/dirtran/trajectory.csv" 
trajectory = np.loadtxt(traj_path, skiprows=1, delimiter=",")
X = np.array([trajectory.T[1], trajectory.T[2], trajectory.T[3], trajectory.T[4]])
U = np.array([trajectory.T[5]])
T = np.array([trajectory.T[0]]).T 
traj_dict = prepare_trajectory(traj_path)

# Getting last rho from time-invariant SOS estimation
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0)
Parser(plant).AddModelFromFile(urdf_path)
plant.Finalize()
tilqr_context = plant.CreateDefaultContext()
input_i = plant.get_actuation_input_port().get_index()
output_i = plant.get_state_output_port().get_index()
plant.get_actuation_input_port().FixValue(tilqr_context, [0])
Q_tilqr = np.diag([10, 10, 1, 1])  
R_tilqr = np.eye(1) * .1
tilqr_context.SetContinuousState(xG)
linearized_cartpole = Linearize(plant, tilqr_context, input_i, output_i,
                                equilibrium_check_tolerance=1e-3) 
(Kf, Sf) = LinearQuadraticRegulator(linearized_cartpole.A(), linearized_cartpole.B(), Q_tilqr, R_tilqr)
hyperparams = {"taylor_deg": 3,
               "lambda_deg": 2}
rhof = bisect_and_verify(sys,Kf,Sf,hyperparams)
print("Last rho from SOS: ", rhof)

# Probabilistic time-varying RoA estimation
traj_x1 = traj_dict["des_cart_pos_list"]
traj_x2 = traj_dict["des_pend_pos_list"]
traj_x3 = traj_dict["des_cart_vel_list"]
traj_x4 = traj_dict["des_pend_vel_list"]
controller_options = {"T_nom": traj_dict["des_time_list"],
                        "U_nom": traj_dict["des_force_list"],
                        "X_nom": np.vstack((traj_x1, traj_x2, traj_x3, traj_x4)),
                        "Q": np.diag([10,10,1,1]),
                        "R": np.array([10]),
                        "xG": xG}
cartpole = {"urdf": urdf_path,
            "sys": sys,
            "x_lim": 0.3}
dt_sim = 0.008  #traj_dict["des_time_list"][1] - traj_dict["des_time_list"][0]
roaConf = {'rho00': 10,
           'rho_f': rhof,
           'nSimulations': 100,
           'dt_sim': dt_sim
           }
sim = StepSimulator(cartpole, controller_options)
estimator = probTVROA(roaConf,sim,verbose = False)
start = time()
print("Time varying estimation, it could take a while (10 min) ...")
(rho, S) = estimator.doEstimate()
funnel_path = "data/cart_pole/funnels/Probfunnel.csv"
print("The estimated rho is: ", rho)
est_time = int(time()-start)
print("Seconds needed: ", est_time)

# Store the obtained funnel
storeFunnel(S,rho,T,funnel_path)

# Show the obtained funnel
plot_indeces = (1,3) # Meaningful values (0,1) (0,2) (0,3) (1,2) (1,3) (2,3)
ticksSize = 20
fontSize = 20
plotFunnel(funnel_path, traj_path, plot_indeces, fontSize = fontSize, ticksSize = ticksSize)

plt.show()