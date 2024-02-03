import cma
from cma.fitness_transformations import EvalParallel2
import numpy as np
import os
import time
from csv import writer

from cart_pole.model.parameters import Cartpole
from cart_pole.controllers.lqr.RoAest.utils import vol_ellipsoid, storeEllipse, ellipseVolume_convexHull, getEllipseFromCsv
from cart_pole.controllers.lqr.RoAest.plots import plot_ellipse
from cart_pole.utilities.process_data import prepare_trajectory
from cart_pole.controllers.lqr.RoAest.SOSest import bisect_and_verify
from cart_pole.trajectory_optimization.dirtrel.dirtrelTrajOpt import RobustDirtranTrajectoryOptimization
from cart_pole.trajectory_optimization.dirtran.dirtranTrajOpt import DirtranTrajectoryOptimization
from cart_pole.simulation.simulator import StepSimulator
from cart_pole.controllers.tvlqr.RoAest.PROBest import probTVROA
from cart_pole.controllers.tvlqr.RoAest.utils import funnelVolume_convexHull, storeFunnel
from cart_pole.model.parameters import generateUrdf

from pydrake.all import Linearize, \
                        LinearQuadraticRegulator, \
                        DiagramBuilder, \
                        AddMultibodyPlantSceneGraph, \
                        Parser

import signal
class timeout:
    def __init__(self, seconds=1, error_message='Timeout'):
        self.seconds = seconds
        self.error_message = error_message
    def handle_timeout(self, signum, frame):
        raise TimeoutError(self.error_message)
    def __enter__(self):
        signal.signal(signal.SIGALRM, self.handle_timeout)
        signal.alarm(self.seconds)
    def __exit__(self, type, value, traceback):
        signal.alarm(0)

def roaVolComputation(sys, traj_path, funnel_path, options):

    # Time invarying RoA estimation
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0)
    Parser(plant).AddModelFromFile(options["urdf"])
    plant.Finalize()
    tilqr_context = plant.CreateDefaultContext()
    input_i = plant.get_actuation_input_port().get_index()
    output_i = plant.get_state_output_port().get_index()
    plant.get_actuation_input_port().FixValue(tilqr_context, [0])
    tilqr_context.SetContinuousState(options["xG"])
    linearized_cartpole = Linearize(plant, tilqr_context, input_i, output_i,
                                    equilibrium_check_tolerance=1e-3) 
    (Kf, Sf) = LinearQuadraticRegulator(linearized_cartpole.A(), linearized_cartpole.B(), options["QN"], np.array([options["R"]]))
    hyperparams = {"taylor_deg": 3,
                "lambda_deg": 2}
    rhof = bisect_and_verify(sys,Kf,Sf,hyperparams)    

    # Probabilistic time varying RoA est
    traj_dict = prepare_trajectory(traj_path)
    traj_x1 = traj_dict["des_cart_pos_list"]
    traj_x2 = traj_dict["des_pend_pos_list"]
    traj_x3 = traj_dict["des_cart_vel_list"]
    traj_x4 = traj_dict["des_pend_vel_list"]
    controller_options = {"T_nom": traj_dict["des_time_list"],
                            "U_nom": traj_dict["des_force_list"],
                            "X_nom": np.vstack((traj_x1, traj_x2, traj_x3, traj_x4)),
                            "Q": options["Q"],
                            "R": np.array([options["R"]]),
                            "xG": options["xG"]}
    cartpole = {"urdf": options["urdf"],
                "sys": sys,
                "x_lim": options["cart_pos_lim"]}
    sim = StepSimulator(cartpole, controller_options)
    roaConf = {'rho00': 10,
            'rho_f': rhof,
            'nSimulations': 100,
            'dt_sim': 0.008}
    estimator = probTVROA(roaConf,sim)

    #with timeout(2000):
    (rho, S) = estimator.doEstimate()
          
    # Store the funnel and compute the volume of the funnel with the convex hull formulation
    storeFunnel(S,rho,sim.T_nom,funnel_path)
    RoA_volume = funnelVolume_convexHull(funnel_path, traj_path)

    return RoA_volume

class CMAES_Opt():
    def __init__(self, params, cost, results_dir, verbose = False):
        ''' Assume params = dict{sys, maxfevals,q_bound,r_bound,q11,q22,r}'''
        self.initial_par = np.array([params["q11"],params["q22"],params["r"]])
        self.max_fevals = params["maxfevals"]
        self.q_bound = params["q_bound"]
        self.r_bound = params["r_bound"]
        self.sys = params["sys"]
        self.urdf = params["urdf"]
        self.verbose = verbose
        self.cost = cost
        self.RoA_path = results_dir+"/Sosfunnel_CMAES.csv" 
        self.traj_path = results_dir+"/trajectory_CMAES.csv"
        self.optimal_traj_path = results_dir+"/trajectoryOptimal_CMAES.csv"
        self.optimal_data_path = results_dir+"/CooptData_CMAES.csv"
        self.results_dir = results_dir
        if self.cost == "volumeDIRTREL":
            self.objective_value_fail = -0.0001 # volume in case of failure
        elif self.cost == "volumeDIRTRAN":
            self.objective_value_fail = -0.0001 # volume in case of failure
        elif self.cost == "lwDIRTREL":
            self.objective_value_fail = 100000 # cost in case of failure
        else:
            print("Wrong cost option")
            assert False
            
 
    def objectiveFunction(self,optimized_par):

        # Solving direct transcription
        options = {"N": 201,
                "x0": [0, np.pi, 0, 0],
                "xG": [0, 0, 0, 0],
                "hBounds": [0.01, 0.06],
                "fl": 6,
                "cart_pos_lim": 0.3,
                "QN": np.diag([100, 100, 100, 100]),
                "R": optimized_par[2],
                "Q": np.diag([optimized_par[0],optimized_par[1], 1, 1]),
                "time_penalization": 0,
                "urdf": self.urdf,
                "Rl": .1,
                "QNl": np.diag([10, 10, .1, .1]),
                "Ql": np.diag([10, 10, .1, .1]),
                "D": 0.2*0.2, 
                "E1": np.zeros((4,4)),
                "tf0": 8}
        if self.cost == "volumeDIRTREL" or self.cost == "lwDIRTREL":
            trajOpt = RobustDirtranTrajectoryOptimization(self.sys, options)
        elif self.cost == "volumeDIRTRAN":
            trajOpt = DirtranTrajectoryOptimization(self.sys,options)
        try:    
            with timeout(seconds=300):    
                T, X, U = trajOpt.ComputeTrajectory()
            traj_data = np.vstack((T, X[0], X[1], X[2], X[3], U)).T
            np.savetxt(self.traj_path, traj_data, delimiter=',',
                        header="time,pos,vel,torque", comments="")
        except:
            if self.verbose:
                print("TrajOpt ERROR, The new " +self.cost+ " is: ", self.objective_value_fail) 
            return self.objective_value_fail

        # Stabilizability check TODO

        # Computation of the objective function
        if self.cost == "volumeDIRTREL" or self.cost == "volumeDIRTRAN":
            try:
                objective_value = -roaVolComputation(self.sys, self.traj_path, self.RoA_path, options)
            except:
                if self.verbose:
                    print("RoA estimation ERROR, The new " +self.cost+ " is: ", self.objective_value_fail) 
                return self.objective_value_fail
        else:
            objective_value = trajOpt.getOptimalCost()
    
        # Saving the ungoing optimization data
        controller_data = [optimized_par[0], optimized_par[1], optimized_par[2],objective_value]
        csvfile = open(self.optimal_data_path, 'a')
        wr = writer(csvfile)
        wr.writerow(np.array(controller_data))
        csvfile.close()

        # Verbose optimization print
        if self.verbose:
            print("Inner optimization function evaluation (q11, q22, r, obj): ", controller_data) 

        return objective_value
    
    def solve(self, sigma0=3,
                    popsize_factor=3,
                    maxfevals=1000,
                    tolfun=1e-11,
                    tolx=1e-11,
                    tolstagnation=100,
                    num_proc=1,
                    sd = "data/cart_pole/outcmaes/"):

        # Define the optimization options and constraints
        bounds = np.array([self.q_bound,self.q_bound,self.r_bound]).T
        sd = self.results_dir+"/outcmaes/"
        opts = cma.CMAOptions()
        opts.set("bounds", list(bounds))
        opts.set("verbose", -3)
        opts.set("popsize_factor", popsize_factor)
        opts.set("verb_filenameprefix", sd)
        opts.set("tolfun", tolfun)
        opts.set("maxfevals", maxfevals)
        opts.set("tolx", tolx)
        opts.set("tolstagnation", tolstagnation)

        if num_proc > 1:
            es = cma.CMAEvolutionStrategy(self.initial_par,
                                    sigma0,
                                    opts)
            start = time.time()
            with EvalParallel2(self.objectiveFunction, num_proc) as eval_all:
                while not es.stop():
                    X = es.ask()
                    es.tell(X, eval_all(X))
                    es.disp()
                    es.logger.add() 
        else:
            es = cma.CMAEvolutionStrategy(self.initial_par,
                                    sigma0,
                                    opts)
            start = time.time()
            es.optimize(self.objectiveFunction)
        optimization_time = int((time.time() - start)/60)

        # Computing and saving the optimal trajectory
        optimal_options = {"N": 201,
                            "x0": [0, np.pi, 0, 0],
                            "xG": [0, 0, 0, 0],
                            "hBounds": [0.01, 0.06],
                            "fl": 6,
                            "cart_pos_lim": 0.3,
                            "QN": np.diag([100, 100, 100, 100]),
                            "R": es.result.xbest[2],
                            "Q": np.diag([es.result.xbest[0],es.result.xbest[1], 1, 1]),
                            "time_penalization": 0,
                            "urdf": self.urdf,
                            "Rl": .1,
                            "QNl": np.diag([10, 10, .1, .1]),
                            "Ql": np.diag([10, 10, .1, .1]),
                            "D": 0.2*0.2, 
                            "E1": np.zeros((4,4)),
                            "tf0": 8}
        if self.cost == "volumeDIRTREL" or self.cost == "lwDIRTREL":
            trajOpt = RobustDirtranTrajectoryOptimization(self.sys, optimal_options)
        elif self.cost == "volumeDIRTRAN":
            trajOpt = DirtranTrajectoryOptimization(self.sys, optimal_options)
        
        try:
            T, X, U = trajOpt.ComputeTrajectory()
            traj_data = np.vstack((T, X[0], X[1], X[2], X[3], U)).T
            np.savetxt(self.optimal_traj_path, traj_data, delimiter=',',
                        header="time,pos,vel,torque", comments="")
        except:
            if self.verbose:
                print("No trajectory founded, probably due to a bad design...")

        # Print the result
        if self.verbose:
            print('The process took %d minutes' % optimization_time)
            print('Optimal solution: ', es.result.xbest)
            print('Optimal value of the objective function: ', es.result.fbest)
        return es.result.xbest, es.result.fbest

if __name__ == "__main__":  
    import argparse
    from datetime import datetime

    parser = argparse.ArgumentParser(description='Cost choice.')
    parser.add_argument("-cost", help="Optimize the DIRTRAN volume(volumeDIRTRAN) or the DIRTREL volume(volumeDIRTREL) or the DIRTREL cost function(lwDIRTREL).")
    args = parser.parse_args()   

    date = datetime.now().strftime("%d%m%Y-%H:%M:%S")
    results_dir = "data/cart_pole/optCMAES_"+date+"_"+args.cost 
    if not os.path.exists(results_dir):
        os.makedirs(results_dir) 
    
    if args.cost == "lwDIRTREL":
        max_f_eval = 300
    else:
        max_f_eval = 1

    sys = Cartpole("short")
    old_Mp = sys.Mp
    sys.Mp = 0.227
    sys.Jp = sys.Jp + (sys.Mp-old_Mp)*(sys.lp**2)
    sys.fl = 6
    urdf_path = generateUrdf(sys.Mp, sys.lp, sys.Jp)
    optimization_params = {"sys": sys,
                "urdf": urdf_path, 
                "xG": [0,0,0,0],
                "maxfevals": max_f_eval,
                "q_bound": [1,20],
                "r_bound": [5,15], 
                "q11": 10,
                "q22": 10,
                "r": 10}
    cost = args.cost
    cmaes = CMAES_Opt(optimization_params, cost, results_dir = results_dir, verbose = True)
    solution, fbest = cmaes.solve(num_proc = 2, maxfevals=optimization_params["maxfevals"])
    Q_opt = np.diag([solution[0], solution[1],1,1])
    R_opt = solution[2]
    print("The optimal Q is: ", Q_opt)
    print("The optimal R is: ", [R_opt])

    RoA_path = results_dir+"/RoA_CMAES.csv"
    traj_path = cmaes.optimal_traj_path
    roa_options = {"QN": np.diag([100,100,100,100]),
                   "Q": Q_opt,
                   "R": R_opt,
                   "urdf": optimization_params["urdf"],
                   "xG": optimization_params["xG"],
                   "cart_pos_lim": 0.3}
    volume = roaVolComputation(sys,traj_path,RoA_path,roa_options)

    init_RoA_path = results_dir+"/initRoA_CMAES.csv"
    init_traj_path = "data/cart_pole/dirtran/trajectory.csv"
    roa_options = {"QN": np.diag([100,100,100,100]),
                   "Q": np.diag([optimization_params["q11"],optimization_params["q22"],1,1]),
                   "R": optimization_params["r"],
                   "urdf": optimization_params["urdf"],
                   "xG": optimization_params["xG"],
                   "cart_pos_lim": 0.3}
    init_volume = roaVolComputation(sys,init_traj_path,init_RoA_path,roa_options)

    print("Volume of CMA-ES funnel:", volume)
    print("Volume of init funnel:", init_volume)