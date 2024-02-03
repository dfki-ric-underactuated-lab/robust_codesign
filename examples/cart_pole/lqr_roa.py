import numpy as np
import matplotlib as mpl
mpl.use("WebAgg")
import matplotlib.pyplot as plt

from cart_pole.model.parameters import Cartpole
from cart_pole.controllers.lqr.RoAest.SOSest import bisect_and_verify
from cart_pole.controllers.lqr.RoAest.plots import get_ellipse_patch

from pydrake.all import Linearize, \
                        LinearQuadraticRegulator, \
                        DiagramBuilder, \
                        AddMultibodyPlantSceneGraph, \
                        Parser

# Cart-pole system init
sys = Cartpole("short")
urdf_path = "data/cart_pole/urdfs/cartpole.urdf"

# Create drake simulation plant from urdf
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0)
Parser(plant).AddModelFromFile(urdf_path)
plant.Finalize()

# Compute drake lqr controller
Q_tilqr = np.diag([10., 100., .1, .1])  
R_tilqr = np.eye(1) * 1
xG = [0,0,0,0]
tilqr_context = plant.CreateDefaultContext()
input_i = plant.get_actuation_input_port().get_index()
output_i = plant.get_state_output_port().get_index()
plant.get_actuation_input_port().FixValue(tilqr_context, [0])
tilqr_context.SetContinuousState(xG)
linearized_cartpole = Linearize(plant, tilqr_context, input_i, output_i,
                                equilibrium_check_tolerance=1e-3) 
(Kf, Sf) = LinearQuadraticRegulator(linearized_cartpole.A(), linearized_cartpole.B(), Q_tilqr, R_tilqr)

# Time invariant RoA estimation
hyperparams = {"taylor_deg": 3,
               "lambda_deg": 2}
rhof = bisect_and_verify(sys,Kf,Sf,hyperparams)
print("")
print("Last rho from SOS: ", rhof)
print("")

# ROA visualization
indexes = (0,1) # Meaningful values (0,1) (0,2) (0,3) (1,2) (1,3) (2,3)
p = get_ellipse_patch(indexes[0], indexes[1], xG,rhof,Sf, linec="green")  
fig, ax = plt.subplots()
ax.add_patch(p)
ax.scatter(xG[indexes[0]], xG[indexes[1]],color="black",marker="o")

plt.show()