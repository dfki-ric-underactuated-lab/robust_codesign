from rtc_CMAES import *

class CMAES_DesignOpt():
    def __init__(self, params, cost,results_dir, verbose = False):
        ''' Assume params = dict{lp, Mp, maxfevals,q_bound,r_bound,m_bound,l_bound,q11,q22,r}'''
        self.initial_par = np.array([params["Mp"], params["lp"]])
        self.q11 = params["q11"]
        self.q22 = params["q22"]
        self.r = params["r"] 
        self.max_fevals = params["maxfevals"]
        self.q_bound = params["q_bound"]
        self.r_bound = params["r_bound"]
        self.m_bound = params["m_bound"]
        self.l_bound = params["l_bound"]
        self.verbose = verbose
        self.cost = cost
        self.RoA_path = results_dir+"/Sosfunnel_CMAES.csv" 
        self.traj_path = results_dir+"/trajectory_CMAES.csv"
        self.optimal_traj_path = results_dir+"/trajectoryOptimal_CMAES.csv"
        self.optimal_data_path = results_dir+"/fullCooptData_CMAES.csv"
        self.results_dir = results_dir          
 
    def objectiveFunction(self,optimized_par):

        # Traj opt and traj stab via cma-es
        sys = Cartpole("short")
        sys.fl = 6
        old_Mp = sys.Mp
        old_lp = sys.lp
        sys.Mp = optimized_par[0]
        sys.lp = optimized_par[1]
        l_ratio = (sys.lp/old_lp)**2
        sys.Jp = sys.Jp*l_ratio + (sys.Mp-old_Mp)*(sys.lp**2)
        urdf_path = generateUrdf(sys.Mp, sys.lp, sys.Jp)
        inner_opt_params = {"sys": sys,
                    "urdf": urdf_path, 
                    "xG": [0,0,0,0],
                    "maxfevals": self.max_fevals,
                    "q_bound": self.q_bound,
                    "r_bound": self.r_bound, 
                    "q11": self.q11,
                    "q22": self.q22,
                    "r": self.r}
        cmaes = CMAES_Opt(inner_opt_params, self.cost, self.results_dir, verbose = True)
        solution, objective_value = cmaes.solve(num_proc = 3, maxfevals=optimization_params["maxfevals"])
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
            print("Outern optimization function evaluation (Mp,lp,q11, q22, r, obj): ", controller_data) 

        return objective_value
    
    def solve(self, sigma0=0.1,
                    popsize_factor=3,
                    maxfevals=1000,
                    tolfun=1e-11,
                    tolx=1e-11,
                    tolstagnation=100,
                    num_proc=1,
                    sd = "data/simple_pendulum/dirtrel/outcmaes/"):

        # Define the optimization options and constraints
        bounds = np.array([self.m_bound,self.l_bound]).T
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

        # Loading optimal solution and computing optimal trajectory
        controller_data = np.loadtxt(self.optimal_data_path, skiprows=1, delimiter=",")
        max_idx = np.where(-controller_data.T[5] == max(-controller_data.T[5]))[0][0]
        m_opt = controller_data[max_idx,0]
        l_opt = controller_data[max_idx,1]
        Q_opt = np.diag([controller_data[max_idx,2],controller_data[max_idx,3],1,1])
        R_opt = [controller_data[max_idx,4]]
        optimal_solution = [l_opt, m_opt, controller_data[max_idx,2],controller_data[max_idx,3], controller_data[max_idx,4]]
        opt_sys = Cartpole("short")
        opt_sys.fl = 6
        old_Mp = opt_sys.Mp
        old_lp = opt_sys.lp
        opt_sys.Mp = m_opt
        opt_sys.lp = l_opt
        l_ratio = (opt_sys.lp/old_lp)**2
        opt_sys.Jp = opt_sys.Jp*l_ratio + (opt_sys.Mp-old_Mp)*(opt_sys.lp**2)
        urdf_path = generateUrdf(opt_sys.Mp, opt_sys.lp, opt_sys.Jp)
        optimal_options = {"N": 201,
                            "x0": [0, np.pi, 0, 0],
                            "xG": [0, 0, 0, 0],
                            "hBounds": [0.01, 0.06],
                            "fl": 6,
                            "cart_pos_lim": 0.3,
                            "QN": np.diag([100,100,100,100]),
                            "R": R_opt[0],
                            "Q": Q_opt,
                            "time_penalization": 0,
                            "urdf": urdf_path,
                            "Rl": .1,
                            "QNl": np.diag([10, 10, .1, .1]),
                            "Ql": np.diag([10, 10, .1, .1]),
                            "D": 0.2*0.2, 
                            "E1": np.zeros((4,4)),
                            "tf0": 8}
        if self.cost == "volumeDIRTREL" or self.cost == "lwDIRTREL":
            trajOpt = RobustDirtranTrajectoryOptimization(opt_sys, optimal_options)
        elif self.cost == "volumeDIRTRAN":
            trajOpt = DirtranTrajectoryOptimization(opt_sys, optimal_options)
        
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
            print('Optimal solution (l, m, q11, q22, r): ', optimal_solution)
            print('Optimal value of the objective function: ', es.result.fbest)
        return optimal_solution, es.result.fbest

if __name__ == "__main__":  
    import argparse
    from datetime import datetime

    parser = argparse.ArgumentParser(description='Cost choice.')
    parser.add_argument("-cost", help="Optimize the DIRTRAN volume(volumeDIRTRAN) or the DIRTREL volume(volumeDIRTREL) or the DIRTREL cost function(lwDIRTREL).")
    args = parser.parse_args()   

    date = datetime.now().strftime("%d%m%Y-%H:%M:%S")
    results_dir = "data/cart_pole/optDesignCMAES_"+date+"_"+args.cost 
    if not os.path.exists(results_dir):
        os.makedirs(results_dir) 
    
    if args.cost == "lwDIRTREL":
        max_f_eval = 300
    else:
        max_f_eval = 1

    optimization_params = {"xG": [0,0,0,0],
                "maxfevals": max_f_eval,
                "q_bound": [1,20],
                "r_bound": [5,15],
                "l_bound": [Cartpole("short").lp,0.3], 
                "m_bound": [Cartpole("short").Mp,0.3],
                "q11": 10,
                "q22": 10,
                "r": 10,
                "Mp":0.227,
                "lp":Cartpole("short").lp}
    cost = args.cost
    design_cmaes = CMAES_DesignOpt(optimization_params, cost,results_dir, verbose = True)
    solution, fbest = design_cmaes.solve(num_proc = 1, maxfevals=1)
    m_opt = solution[0]
    l_opt = solution[1]
    Q_opt = np.diag([solution[2], solution[3],1,1])
    R_opt = solution[4]
    print("The optimal l is: ", l_opt)
    print("The optimal m is: ", m_opt)
    print("The optimal Q is: ", Q_opt)
    print("The optimal R is: ", R_opt)

    opt_sys = Cartpole("short")
    opt_sys.fl = 6
    old_Mp = opt_sys.Mp
    old_lp = opt_sys.lp
    opt_sys.Mp = m_opt
    opt_sys.lp = l_opt
    l_ratio = (opt_sys.lp/old_lp)**2
    opt_sys.Jp = opt_sys.Jp*l_ratio + (opt_sys.Mp-old_Mp)*(opt_sys.lp**2)
    urdf_path = generateUrdf(opt_sys.Mp, opt_sys.lp, opt_sys.Jp)
    RoA_path = results_dir+"/RoA_CMAES.csv"
    traj_path = design_cmaes.optimal_traj_path
    roa_options = {"QN": np.diag([100,100,100,100]),
                   "Q": Q_opt,
                   "R": R_opt,
                   "urdf": urdf_path,
                   "xG": optimization_params["xG"],
                   "cart_pos_lim": 0.3}
    volume = roaVolComputation(opt_sys,traj_path,RoA_path,roa_options)

    sys = Cartpole("short")
    sys.fl = 6
    old_Mp = sys.Mp
    old_lp = sys.lp
    sys.Mp = optimization_params["Mp"]
    sys.lp = optimization_params["lp"]
    l_ratio = (sys.lp/old_lp)**2
    sys.Jp = sys.Jp*l_ratio + (sys.Mp-old_Mp)*(sys.lp**2)
    urdf_path = generateUrdf(sys.Mp, sys.lp, sys.Jp)
    init_RoA_path = results_dir+"/initRoA_CMAES.csv"
    init_traj_path = "data/cart_pole/dirtran/trajectory.csv"
    roa_options = {"QN": np.diag([100,100,100,100]),
                   "Q": np.diag([optimization_params["q11"],optimization_params["q22"],1,1]),
                   "R": optimization_params["r"],
                   "urdf": urdf_path,
                   "xG": optimization_params["xG"],
                   "cart_pos_lim": 0.3}
    init_volume = roaVolComputation(sys,init_traj_path,init_RoA_path,roa_options)

    print("Volume of CMA-ES funnel:", volume)
    print("Volume of init funnel:", init_volume)