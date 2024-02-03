from rtc_CMAES import *
from csv import writer

class CMAES_DesignOpt():
    def __init__(self, params, cost, results_dir, verbose = False):
        ''' Assume params = dict{M,maxfevals,q_bound,r_bound,q11,q22,r,b,tl}, M = [m,l] '''
        self.optimization_params = params
        self.initial_par = np.array(params["M"])
        self.q11 = params["q11"]
        self.q22 = params["q22"]
        self.r = params["r"]
        self.max_fevals = params["maxfevals"]
        self.q11_bound = params["q11_bound"]
        self.q22_bound = params["q22_bound"]
        self.r_bound = params["r_bound"]
        self.m_bound = params["m_bound"]
        self.l_bound = params["l_bound"]
        self.damping = params["b"]
        self.torque_limit = params["tl"]
        self.gravity = 9.81
        self.columb_frict = 0.0
        self.verbose = verbose
        self.cost = cost
        self.optimal_data_path = results_dir+"/fullCoopt_CMAES.csv"
        self.optimal_traj_path = results_dir+"/trajectoryOptimal_CMAES.csv"
        self.results_dir = results_dir
 
    def objectiveFunction(self,optimized_par):

        # Traj opt and traj stab via cma-es
        if self.cost == "lwDIRTREL":
            max_f_eval = 300
        else:
            max_f_eval = 40
        inner_opt_par = {"M": optimized_par, # design parameters [m,l]
                "maxfevals": max_f_eval,
                "q11_bound": self.q11_bound,
                "q22_bound": self.q22_bound,
                "r_bound": self.r_bound, 
                "q11": self.q11,
                "q22": self.q22,
                "r": self.r,
                "b": self.damping,
                "tl": self.torque_limit}
        cmaes = CMAES_Opt(inner_opt_par, self.cost, self.results_dir, verbose = self.verbose)
        solution, objective_value = cmaes.solve(num_proc = 10, maxfevals=inner_opt_par["maxfevals"])
        self.q11 = solution[0]
        self.q22 = solution[1]
        self.r = solution[2]

        # Saving the ungoing optimization data
        controller_data = [optimized_par[0], optimized_par[1], solution[0], solution[1], solution[2], objective_value]
        csvfile = open(self.optimal_data_path, 'a')
        wr = writer(csvfile)
        wr.writerow(np.array(controller_data))
        csvfile.close()

        # Verbose optimization print
        if self.verbose:
            print("Outern optimization function evaluation (m, l, q11, q22, r, obj): ", controller_data) 

        return objective_value
    
    def solve(self, sigma0=0.1,
                    popsize_factor=3,
                    maxfevals=1000,
                    tolfun=1e-11,
                    tolx=1e-11,
                    tolstagnation=100,
                    num_proc=1,
                    sd = "data/simple_pendulum/outcmaes/"):

        # Define the optimization options and constraints
        sd = self.results_dir+"/outcmaes/"
        bounds = np.array([self.m_bound,self.l_bound]).T
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

        # Loading optimal solution and computing optimal trajectory
        controller_data = np.loadtxt(design_cmaes.optimal_data_path, skiprows=1, delimiter=",")
        max_idx = np.where(-controller_data.T[5] == max(-controller_data.T[5]))[0][0]
        m_opt = controller_data[max_idx,0]
        l_opt = controller_data[max_idx,1]
        Q_opt = np.diag([controller_data[max_idx,2],controller_data[max_idx,3]])
        R_opt = [controller_data[max_idx,4]]
        optimal_solution = [m_opt, l_opt, controller_data[max_idx,2],controller_data[max_idx,3], controller_data[max_idx,4]]
        opt_mpar = {"l": l_opt,
                "m": m_opt,
                "b": self.damping, 
                "g": self.gravity,
                "cf": self.columb_frict,
                "tl": self.torque_limit}
        opt_options = {"N": 51,
                "R": R_opt[0],
                "Rl": R_opt[0],
                "Q": Q_opt,
                "Ql": Q_opt,
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
            trajOpt = RobustDirtranTrajectoryOptimization(opt_mpar, opt_options)
        elif self.cost == "volumeDIRTRAN":
            trajOpt = DirtranTrajectoryOptimization(opt_mpar, opt_options)
        T, X, U = trajOpt.ComputeTrajectory()
        traj_data = np.vstack((T, X[0], X[1], U)).T
        np.savetxt(self.optimal_traj_path, traj_data, delimiter=',',
                    header="time,pos,vel,torque", comments="")

        # Print the result
        if self.verbose:
            print('The process took %d minutes' % optimization_time)
            print('Optimal solution (m, l, q11, q22, r): ', optimal_solution)
            print('Optimal value of the objective function: ', es.result.fbest)
        return optimal_solution, es.result.fbest

if __name__ == "__main__":  
    import matplotlib as mpl
    mpl.use("WebAgg")
    import matplotlib.pyplot as plt
    from simple_pendulum.trajectory_optimization.dirtrel.dirtrelTrajOpt import RobustDirtranTrajectoryOptimization
    import argparse
    from datetime import datetime

    parser = argparse.ArgumentParser(description='Cost choice.')
    parser.add_argument("-cost", help="Optimize the DIRTRAN volume(volumeDIRTRAN) or the DIRTREL volume(volumeDIRTREL) or the DIRTREL cost function(lwDIRTREL).")
    args = parser.parse_args() 

    date = datetime.now().strftime("%d%m%Y-%H:%M:%S")
    results_dir = "data/simple_pendulum/optDesignCMAES_"+date+"_"+args.cost 
    if not os.path.exists(results_dir):
        os.makedirs(results_dir) 

    optimization_params = {"M": [0.7, 0.4], # design parameters [m,l]
                "maxfevals": 20,
                "q11_bound": [1,10],
                "q22_bound": [1,10],
                "r_bound": [0.001,10], 
                "m_bound": [0.6,0.8], #[0.67,1]
                "l_bound": [0.4,0.6], #[0.2,0.7]
                "q11": 10,
                "q22": 1,
                "r": 0.1,
                "b": 0.1,
                "tl": 2.5}
    cost = args.cost
    design_cmaes = CMAES_DesignOpt(optimization_params, cost,results_dir, verbose = True)
    solution, fbest = design_cmaes.solve(num_proc = 1, maxfevals=optimization_params["maxfevals"])
    m_opt = solution[0]
    l_opt = solution[1]
    Q_opt = np.diag([solution[2],solution[3]])
    R_opt = [solution[4]]
    print("The optimal m is: ", m_opt)
    print("The optimal l is: ", l_opt)
    print("The optimal Q is: ", Q_opt)
    print("The optimal R is: ", R_opt)

    # Pendulum optimized parameters
    opt_mpar = {"l": l_opt, 
        "m": m_opt,
        "b": optimization_params["b"],
        "g": 9.81,
        "cf": 0.0,
        "tl": optimization_params["tl"]}
    traj_path = results_dir+"/trajectoryOptimal_CMAES.csv"
    funnel_path = results_dir+"/SosfunnelOptimal_CMAES.csv"
    roa_options = {"N": 51,
                   "Q": Q_opt,
                   "R": R_opt}
    volume = roaVolComputation(opt_mpar, roa_options, traj_path, funnel_path, time_out = False)
    print("Volume of CMA-ES funnel:", volume)

    # Pendulum parameters
    mpar = {"l": optimization_params["M"][1], 
        "m": optimization_params["M"][0],
        "b": optimization_params["b"],
        "g": 9.81,
        "cf": 0.0,
        "tl": optimization_params["tl"]}
    dirtrel_funnel_path = "data/simple_pendulum/funnels/Sosfunnel.csv"
    dirtrel_traj_path = "data/simple_pendulum/dirtran/trajectory.csv"
    roa_options = {"N": 51,
                   "Q": np.diag([optimization_params["q11"],optimization_params["q22"]]),
                   "R": optimization_params["r"]}
    dirtrel_volume = roaVolComputation(mpar, roa_options, dirtrel_traj_path, dirtrel_funnel_path, time_out = False)
    print("Volume of DIRTRAN funnel:", dirtrel_volume)