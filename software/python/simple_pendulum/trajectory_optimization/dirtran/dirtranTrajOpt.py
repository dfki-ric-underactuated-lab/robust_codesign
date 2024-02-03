import numpy as np

from pydrake.systems.trajectory_optimization import DirectTranscription, TimeStep
from pydrake.trajectories import PiecewisePolynomial
from pydrake.examples.pendulum import PendulumPlant
from pydrake.solvers.snopt import SnoptSolver
from pydrake.symbolic import sin
from pydrake.all import MathematicalProgram, SolverOptions

class DrakeDirtranTrajectoryOptimization():
    def __init__(self, params):
        self.m = params["m"]
        self.l = params["l"]
        self.b = params["b"]
        self.g = params["g"]
        self.cf = params["cf"]
        self.tl = params["tl"]

        # Pendulum initialization
        self.pendulum_plant = PendulumPlant()
        self.pendulum_context = self.pendulum_plant.CreateDefaultContext()
        self.pendulum_params = self.pendulum_plant.get_mutable_parameters(
                                                        self.pendulum_context)
        self.pendulum_params[0] = self.m
        self.pendulum_params[1] = self.l
        self.pendulum_params[2] = self.b
        self.pendulum_params[3] = self.g
        self.torque_limit = self.tl

    def compute_trajectory(self, options):
        self.options = options
        self.R = options["R"]
        self.Q = options["Q"]
        self.QN = options["QN"]
        self.N = options["N"]
        self.tf0 = options["tf0"]
        self.speed_limit = options["speed_limit"]
        self.theta_limit = options["theta_limit"] 

        dirtrel = DirectTranscription(self.pendulum_plant,
                                   self.pendulum_context,
                                   options["N"], fixed_timestep = TimeStep(0.05)) 
        self.prog = dirtrel.prog()

        # Add input constraints and cost
        u = dirtrel.input()
        dirtrel.AddRunningCost(options["R"] * u[0] ** 2)
        dirtrel.AddConstraintToAllKnotPoints(-self.torque_limit <= u[0])
        dirtrel.AddConstraintToAllKnotPoints(u[0] <= self.torque_limit)
        # Add state constraints and costs
        state = dirtrel.state()
        dirtrel.AddRunningCost(options["Q"][0][0] * state[0] ** 2)
        dirtrel.AddRunningCost(options["Q"][1][1] * state[1] ** 2)
        dirtrel.prog().AddBoundingBoxConstraint(options["x0"],
                                                options["x0"],
                                                dirtrel.initial_state())
        dirtrel.prog().AddBoundingBoxConstraint(options["xG"], 
                                                options["xG"], 
                                                dirtrel.final_state())
        dirtrel.AddConstraintToAllKnotPoints(state[1] <= self.speed_limit)
        dirtrel.AddConstraintToAllKnotPoints(-self.speed_limit <= state[1])
        dirtrel.AddConstraintToAllKnotPoints(state[0] <= self.theta_limit)
        dirtrel.AddConstraintToAllKnotPoints(-self.theta_limit <= state[0])
        # Add final cost
        dirtrel.AddFinalCost(dirtrel.time() * options["time_penalization"])
        # Add initial trajectory guess
        init_traj_time_interval = [0, options["tf0"]]
        initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(init_traj_time_interval,
                                                                  np.column_stack((options["x0"], options["xG"])))
        dirtrel.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)

        # Solve the problem
        solver = SnoptSolver()
        result = solver.Solve(dirtrel.prog())
        assert result.is_success()
        times = dirtrel.GetSampleTimes(result)
        inputs = dirtrel.GetInputSamples(result)
        states = dirtrel.GetStateSamples(result)

        return times, states, inputs

class DirtranTrajectoryOptimization():

    def __init__(self, params, options, integrator = "MIDPOINT"):
        self.prog = MathematicalProgram()
        self.options = options
        self.params = params
        self.integrator = integrator
        
        # Setup of the Variables
        self.N = options["N"]
        self.h_vars = self.prog.NewContinuousVariables(self.N-1, "h")
        self.x_vars = np.array([self.prog.NewContinuousVariables(self.N, "x0"),
                                self.prog.NewContinuousVariables(self.N, "x1")])
        self.u_vars = self.prog.NewContinuousVariables(self.N-1, "u")
    
    def ComputeTrajectory(self):
        # Create constraints for dynamics and add them
        self.AddDynamicConstraints(self.options["x0"],self.options["xG"])

        # Add a linear constraint to a variable in all the knot points
        self.AddConstraintToAllKnotPoints(self.h_vars,self.options["hBounds"][0],self.options["hBounds"][1])
        self.AddConstraintToAllKnotPoints(self.u_vars,-self.params["tl"],self.params["tl"])
        self.AddConstraintToAllKnotPoints(self.x_vars[0],-self.options["theta_limit"],self.options["theta_limit"])
        self.AddConstraintToAllKnotPoints(self.x_vars[1],-self.options["speed_limit"],self.options["speed_limit"])

        # Add an initial guess for the resulting trajectory
        self.AddStateInitialGuess(self.options["x0"],self.options["xG"])
        self.AddTimeInitialGuess(self.options["tf0"])

        # Add cost on the final state
        self.AddFinalStateCost(self.options["QN"])

        # Add constraints on the time trajectory
        self.AddTimePenalization(self.h_vars, self.options["time_penalization"])
        self.AddEqualTimeStepsConstraint(self.h_vars)

        # Add integrative cost
        self.AddRunningCost(np.append(np.append(self.x_vars,self.u_vars), self.h_vars))

        # Solve the Mathematical program
        solver = SnoptSolver()
        result = solver.Solve(self.prog)
        solver_options = SolverOptions()
        solver_options.SetOption(solver.id(),'Major Optimality Tolerance', 1e-3)
        solver_options.SetOption(solver.id(),'Major Feasibility Tolerance', 1e-4)
        solver_options.SetOption(solver.id(),'Minor Feasibility Tolerance', 1e-4)
        self.prog.SetSolverOptions(solver_options)

        assert result.is_success()
        times, states, inputs = self.GetResultingTrajectory(result)

        return times, states, inputs 

    def AddDynamicConstraints(self, x0, xG):    
        self.prog.AddConstraint(self.x_vars[0][0] == x0[0])
        self.prog.AddConstraint(self.x_vars[1][0] == x0[1])
        for i in range(self.N-1):
            x_n = [self.x_vars[0][i],self.x_vars[1][i]]
            u_n = self.u_vars[i]
            h_n = self.h_vars[i]
            x_nplus1 = self.dynamics_integration(x_n, u_n, h_n, self.integrator)
            self.prog.AddConstraint(self.x_vars[0][i+1] == x_nplus1[0])
            self.prog.AddConstraint(self.x_vars[1][i+1] == x_nplus1[1])
        self.prog.AddConstraint(self.x_vars[0][-1] == xG[0])
        self.prog.AddConstraint(self.x_vars[1][-1] == xG[1])
    
    def AddConstraintToAllKnotPoints(self, traj_vars, lb, ub):
        lb_vec = np.ones(len(traj_vars))*lb
        ub_vec = np.ones(len(traj_vars))*ub
        self.prog.AddLinearConstraint(traj_vars, lb_vec, ub_vec)

    def AddStateInitialGuess(self, init_, end_):
        init_guess0 = np.linspace(init_[0], end_[0],self.N)
        init_guess1 = np.linspace(init_[1], end_[1],self.N)
        for i in range(self.N):
            self.prog.SetInitialGuess(self.x_vars[0][i], init_guess0[i])
            self.prog.SetInitialGuess(self.x_vars[1][i], init_guess1[i])
    
    def AddTimeInitialGuess(self, tf0):
        init_guess = np.linspace(0, tf0, self.N)
        for i in range(self.N-1):
            self.prog.SetInitialGuess(self.h_vars[i], init_guess[i+1]-init_guess[i])

    def AddFinalStateCost(self, QN):
        x_final = self.x_vars.T[-1]
        self.prog.AddCost(x_final.T.dot(QN.dot(x_final)))

    def AddTimePenalization(self, h_vars, penalty):
        cost = 0
        for i in range(len(h_vars)):
            cost = cost + h_vars[i]
        self.prog.AddCost(penalty*cost)
    
    def AddEqualTimeStepsConstraint(self, traj_vars):
        for i in range(1,len(traj_vars)):
            self.prog.AddConstraint(traj_vars[i-1] == traj_vars[i])

    def running_cost(self,h,x,u):
        Q = h*self.options["Q"]
        R = h*self.options["R"]
        g = .5*(x-self.options["xG"]).dot(Q.dot(x-self.options["xG"])) + .5*R*u**2
        dg = [g, (x-self.options["xG"]).dot(Q), R*u]
        return g

    def AddRunningCost(self, traj_vars):
        N = self.options["N"]

        x_vars = np.array([traj_vars[[i for i in range(N)]],traj_vars[[N+i for i in range(N)]]]).T
        u_vars = np.array(traj_vars[[(2*N + i) for i in range(N-1)]])
        h_vars = np.array(traj_vars[[(3*N -1 + i) for i in range(N-1)]])

        cost = 0
        for i in range(N-1):
            if self.integrator == "EULER":
                cost += self.running_cost(h_vars[i],x_vars[:][i],u_vars[i])
            elif self.integrator == "MIDPOINT":
                cost += self.running_cost(h_vars[i],0.5*(x_vars[:][i]+x_vars[:][i+1]),u_vars[i])
        self.prog.AddCost(cost)

    def GetResultingTrajectory(self, result):
        timeSteps = result.GetSolution(self.h_vars)
        t_prev = 0
        time_traj = [t_prev]
        for h_i in timeSteps:
            time_traj = np.append(time_traj,[t_prev + h_i])
            t_prev = t_prev + h_i
        state_traj = result.GetSolution(self.x_vars)
        input_traj = np.append([0],result.GetSolution(self.u_vars))
        input_traj = np.reshape(input_traj,(1,self.N))

        return time_traj,state_traj,input_traj 

    def dynamics_integration(self,x_n,u_n, h_n, integrator):
        if integrator == "EULER":
            f_n, df_n = self.dynamics_f(x_n,u_n)
            x_nplus1 = np.array(x_n) + h_n*np.array(f_n)
        elif integrator == "MIDPOINT":
            f_n, df_n = self.dynamics_f(x_n,u_n)
            x_mid = np.array(x_n) + (h_n/2)*np.array(f_n)
            f_mid, df_mid = self.dynamics_f(x_mid,u_n)
            x_nplus1 = np.array(x_n) + h_n*np.reshape(f_mid,(2,))
              
        return x_nplus1
    
    def dynamics_f(self, x,u):
        q = x[0]
        qd = x[1]
        
        m = self.params["m"]
        g = self.params["g"]
        l = self.params["l"]
        b = self.params["b"]
        
        qdd = u/(m*l*l) - g*sin(q)/l - b*qd/(m*l*l)
        f = [qd,qdd]
        df = [] # Not used
        return f, df