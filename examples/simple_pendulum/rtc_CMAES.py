import cma
from cma.fitness_transformations import EvalParallel2
import numpy as np
import os
import time
from csv import writer

from simple_pendulum.controllers.tvlqr.roa.utils import funnelVolume_convexHull
from simple_pendulum.utilities.process_data import prepare_trajectory, saveFunnel
from simple_pendulum.controllers.tvlqr.roa.sos import TVsosRhoComputation
from simple_pendulum.controllers.lqr.roa.sos import SOSequalityConstrained
from simple_pendulum.trajectory_optimization.dirtrel.dirtrelTrajOpt import RobustDirtranTrajectoryOptimization
from simple_pendulum.trajectory_optimization.dirtran.dirtranTrajOpt import DirtranTrajectoryOptimization
from simple_pendulum.controllers.tvlqr.tvlqr import TVLQRController
from simple_pendulum.controllers.lqr.lqr_controller import LQRController
from simple_pendulum.model.pendulum_plant import PendulumPlant

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

def roaVolComputation(opt_mpar, options, traj_path, funnel_path = None, verbose = False, time_out = True):

    # load trajectory
    data_dict = prepare_trajectory(traj_path)
    trajectory = np.loadtxt(traj_path, skiprows=1, delimiter=",")
    time = trajectory.T[0].T
    dt = time[1]-time[0]
    x0_traj = [trajectory.T[1].T, trajectory.T[2].T]

    controller = TVLQRController(data_dict=data_dict, mass=opt_mpar["m"], length=opt_mpar["l"],
                             damping=opt_mpar["b"], gravity=opt_mpar["g"],
                             torque_limit=opt_mpar["tl"])
    controller.set_costs(options["Q"], [options["R"]])
    controller.set_goal([np.pi, 0.0])
    pendulum = PendulumPlant(mass=opt_mpar["m"], length=opt_mpar["l"],
                             damping=opt_mpar["b"], gravity=opt_mpar["g"],
                             torque_limit=opt_mpar["tl"])

    # Taking the finals values of S and rho from the invariant case, SOS method has been chosen
    (rhof, Sf) = SOSequalityConstrained(pendulum,LQRController(mass=opt_mpar["m"], length=opt_mpar["l"],
                                                                damping=opt_mpar["b"], gravity=opt_mpar["g"],
                                                                torque_limit=opt_mpar["tl"]))

    # Time-varying RoA estimation, SOS method has been chosen
    if time_out:
        with timeout(seconds=300): 
            (rho, S) = TVsosRhoComputation(pendulum, controller, time, options["N"], rhof, verbose)
    else:
        (rho, S) = TVsosRhoComputation(pendulum, controller, time, options["N"], rhof, verbose)
    
    if funnel_path != None:
        # Save the funnel
        saveFunnel(rho, S, time, funnel_path = funnel_path)      
          
    # Compute the volume of the funnel with the convex hull formulation
    funnel_volume = funnelVolume_convexHull(funnel_path, traj_path)
    return funnel_volume

class CMAES_Opt():
    def __init__(self, params, cost, results_dir, verbose = False):
        ''' Assume params = dict{M,maxfevals,q_bound,r_bound,q11,q22,r,b,tl}, M = [m,l] '''
        self.initial_par = np.array([params["q11"],params["q22"],params["r"]])
        self.max_fevals = params["maxfevals"]
        self.q11_bound = params["q11_bound"]
        self.q22_bound = params["q22_bound"]
        self.r_bound = params["r_bound"]
        self.verbose = verbose
        self.cost = cost
        self.funnel_path = results_dir+"/Sosfunnel_CMAES.csv" 
        self.traj_path = results_dir+"/trajectory_CMAES.csv"
        self.optimal_traj_path = results_dir+"/trajectoryOptimal_CMAES.csv"
        self.optimal_data_path = results_dir+"/CooptData_CMAES.csv"
        self.results_dir = results_dir
        self.id = 0
        
        # Pendulum parameters
        self.mpar = {"l": params["M"][1],
                "m": params["M"][0],
                "b": params["b"], 
                "g": 9.81,
                "cf": 0.0,
                "tl": params["tl"]}

        if self.cost == "volumeDIRTREL":
            self.objective_value_fail = -0.001 # volume in case of failure
        elif self.cost == "lwDIRTREL":
            self.objective_value_fail = 100000 # cost in case of failure
        elif self.cost == "volumeDIRTRAN":
            self.objective_value_fail = -0.001 # volume in case of failure
        else:
            print("Wrong cost option")
            assert False
 
    def objectiveFunction(self,optimized_par):

        # Solving direct transcription
        options = {"N": 51,
                "R": optimized_par[2],
                "Rl": optimized_par[2],
                "Q": np.diag([optimized_par[0],optimized_par[1]]),
                "Ql": np.diag([optimized_par[0],optimized_par[1]]),
                "QN": np.eye(2)*100,
                "QNl": np.eye(2)*100,
                "D": 0.2*0.2, 
                "E1": np.zeros((2,2)), 
                "x0": [0.0,0.0],
                "xG": [np.pi, 0.0],
                "tf0": 3,
                "speed_limit": 7,
                "theta_limit": 2*np.pi,
                "time_penalization": 0.1, 
                "hBounds": [0.01, 0.1]}
        if self.cost == "volumeDIRTREL" or self.cost == "lwDIRTREL":
            trajOpt = RobustDirtranTrajectoryOptimization(self.mpar, options)
        elif self.cost == "volumeDIRTRAN":
            trajOpt = DirtranTrajectoryOptimization(self.mpar,options)

        try:     
            with timeout(60):   
                T, X, U = trajOpt.ComputeTrajectory()
            traj_data = np.vstack((T, X[0], X[1], U)).T
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
                objective_value = -roaVolComputation(self.mpar, options, self.traj_path, self.funnel_path)
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
    
    def solve(self, sigma0=0.9,
                    popsize_factor=3,
                    maxfevals=1000,
                    tolfun=1e-11,
                    tolx=1e-11,
                    tolstagnation=100,
                    num_proc=1,
                    sd = "data/simple_pendulum/dirtrel/outcmaes/"):

        # Define the optimization options and constraints
        bounds = np.array([self.q11_bound,self.q22_bound,self.r_bound]).T
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
        optimal_options = {"N": 51,
                "R": es.result.xbest[2],
                "Rl": es.result.xbest[2],
                "Q": np.diag([es.result.xbest[0],es.result.xbest[1]]),
                "Ql": np.diag([es.result.xbest[0],es.result.xbest[1]]),
                "QN": np.eye(2)*100,
                "QNl": np.eye(2)*100,
                "D": 0.2*0.2, 
                "E1": np.zeros((2,2)), 
                "x0": [0.0,0.0],
                "xG": [np.pi, 0.0],
                "tf0": 3,
                "speed_limit": 7,
                "theta_limit": 2*np.pi,
                "time_penalization": 0.1, 
                "hBounds": [0.01, 0.1]}
        if self.cost == "volumeDIRTREL" or self.cost == "lwDIRTREL":
            trajOpt = RobustDirtranTrajectoryOptimization(self.mpar, optimal_options)
        elif self.cost == "volumeDIRTRAN":
            trajOpt = DirtranTrajectoryOptimization(self.mpar, optimal_options)
        
        try:
            T, X, U = trajOpt.ComputeTrajectory()
            traj_data = np.vstack((T, X[0], X[1], U)).T
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
    results_dir = "data/simple_pendulum/optCMAES_"+date+"_"+args.cost 
    if not os.path.exists(results_dir):
        os.makedirs(results_dir) 
    
    if args.cost == "lwDIRTREL":
        max_f_eval = 300
    else:
        max_f_eval = 100

    optimization_params = {"M": [0.7, 0.4], # design parameters [m,l]
                "maxfevals": max_f_eval,
                "q11_bound": [1,10],
                "q22_bound": [1,10],
                "r_bound": [.001,10], 
                "q11": 10,
                "q22": 1,
                "r": .1,
                "b": 0.1,
                "tl": 2.5} 
    cost = args.cost
    cmaes = CMAES_Opt(optimization_params, cost, results_dir = results_dir, verbose = True)
    solution, fbest = cmaes.solve(num_proc = 10, maxfevals=optimization_params["maxfevals"])
    Q_opt = np.diag([solution[0], solution[1]])
    R_opt = solution[2]
    print("The optimal Q is: ", Q_opt)
    print("The optimal R is: ", [R_opt])

    # Pendulum parameters
    mpar = {"l": optimization_params["M"][1], 
        "m": optimization_params["M"][0],
        "b": optimization_params["b"],
        "g": 9.81,
        "cf": 0.0,
        "tl": optimization_params["tl"]}

    traj_path = results_dir+"/trajectoryOptimal_CMAES.csv"
    funnel_path = results_dir+"/SosfunnelOptimal_CMAES.csv"
    roa_options = {"N": 51,
                   "Q": Q_opt,
                   "R": R_opt}
    volume = roaVolComputation(mpar, roa_options, traj_path, funnel_path)
    print("Volume of CMA-ES funnel:", volume)

    dirtrel_funnel_path = "data/simple_pendulum/funnels/SosfunnelDIRTRAN.csv"
    dirtrel_traj_path = "data/simple_pendulum/dirtran/trajectory.csv"
    roa_options = {"N": 51,
                   "Q": np.diag([optimization_params["q11"],optimization_params["q22"]]),
                   "R": optimization_params["r"]}
    dirtrel_volume = roaVolComputation(mpar, roa_options, dirtrel_traj_path, dirtrel_funnel_path, time_out = False)
    print("Volume of DIRTRAN funnel:", dirtrel_volume)