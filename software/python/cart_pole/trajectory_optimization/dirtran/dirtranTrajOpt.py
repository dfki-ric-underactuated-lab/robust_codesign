import numpy as np

from pydrake.solvers.snopt import SnoptSolver
from pydrake.all import MathematicalProgram

import numpy as np

from pydrake.solvers.snopt import SnoptSolver
from pydrake.all import MathematicalProgram, SolverOptions

class DirtranTrajectoryOptimization():

    def __init__(self, sys, options, integrator = "MIDPOINT"):
        self.prog = MathematicalProgram()
        self.sys = sys
        self.options = options
        self.integrator = integrator

        # Setup of the Variables
        self.N = options["N"]
        self.h_vars = self.prog.NewContinuousVariables(self.N-1, "h")
        self.x_vars = np.array([self.prog.NewContinuousVariables(self.N, "x1"),
                                self.prog.NewContinuousVariables(self.N, "x2"),
                                self.prog.NewContinuousVariables(self.N, "x3"),
                                self.prog.NewContinuousVariables(self.N, "x4")])
        self.u_vars = self.prog.NewContinuousVariables(self.N-1, "u")
    
    def ComputeTrajectory(self):
        # Create constraints for dynamics and add them
        self.AddDynamicConstraints(self.options["x0"], self.options["xG"])

        # Add a linear constraint to a variable in all the knot points
        self.AddConstraintToAllKnotPoints(self.h_vars, self.options["hBounds"][0], self.options["hBounds"][1])
        self.AddConstraintToAllKnotPoints(self.u_vars, -self.options["fl"], self.options["fl"])
        self.AddConstraintToAllKnotPoints(self.x_vars[0], -self.options["cart_pos_lim"], self.options["cart_pos_lim"])

        # Add an initial guess for the resulting trajectory
        self.AddStateInitialGuess(self.options["x0"], self.options["xG"])
        self.AddTimeInitialGuess(self.options["tf0"])

        # Add cost on the final state
        #self.AddFinalStateCost(self.options["QN"])

        # Add constraints on the time trajectory
        self.AddTimePenalization(self.h_vars, self.options["time_penalization"])
        self.AddEqualTimeStepsConstraint(self.h_vars)

        # Add integrative cost
        self.AddRunningCost(np.append(np.append(self.x_vars,self.u_vars), self.h_vars))

        # Solve the Mathematical program
        solver = SnoptSolver()
        solver_options = SolverOptions()
        solver_options.SetOption(solver.id(),'Major Optimality Tolerance', 1e-4)
        solver_options.SetOption(solver.id(),'Major Feasibility Tolerance', 1e-4)
        solver_options.SetOption(solver.id(),'Minor Feasibility Tolerance', 1e-4)
        self.prog.SetSolverOptions(solver_options)

        #print('Solver Engaged')
        result = solver.Solve(self.prog)
        assert result.is_success()
        times, states, inputs = self.GetResultingTrajectory(result)

        return times, states, inputs 

    def AddDynamicConstraints(self, x0, xG):    
        self.prog.AddConstraint(self.x_vars[0][0] == x0[0])
        self.prog.AddConstraint(self.x_vars[1][0] == x0[1])
        self.prog.AddConstraint(self.x_vars[2][0] == x0[2])
        self.prog.AddConstraint(self.x_vars[3][0] == x0[3])
        for i in range(self.N-1):
            x_n = [self.x_vars[0][i], self.x_vars[1][i], self.x_vars[2][i], self.x_vars[3][i]]
            u_n = [self.u_vars[i]]
            h_n = self.h_vars[i]
            x_nplus1, dx_nplus1 = self.dynamics_integration(x_n, u_n, h_n, self.integrator)
            self.prog.AddConstraint(self.x_vars[0][i+1] == x_nplus1[0])
            self.prog.AddConstraint(self.x_vars[1][i+1] == x_nplus1[1])
            self.prog.AddConstraint(self.x_vars[2][i+1] == x_nplus1[2])
            self.prog.AddConstraint(self.x_vars[3][i+1] == x_nplus1[3])
        self.prog.AddConstraint(self.x_vars[0][-1] == xG[0])
        self.prog.AddConstraint(self.x_vars[1][-1] == xG[1])
        self.prog.AddConstraint(self.x_vars[2][-1] == xG[2])
        self.prog.AddConstraint(self.x_vars[3][-1] == xG[3])
    
    def AddConstraintToAllKnotPoints(self, traj_vars, lb, ub):
        lb_vec = np.ones(len(traj_vars))*lb
        ub_vec = np.ones(len(traj_vars))*ub
        self.prog.AddLinearConstraint(traj_vars, lb_vec, ub_vec)

    def AddStateInitialGuess(self, init_, end_):
        init_guess1 = np.linspace(init_[0], end_[0], self.N)
        init_guess2 = np.linspace(init_[1], end_[1], self.N)
        init_guess3 = np.linspace(init_[2], end_[2], self.N)
        init_guess4 = np.linspace(init_[3], end_[3], self.N)
        for i in range(self.N):
            self.prog.SetInitialGuess(self.x_vars[0][i], init_guess1[i])
            self.prog.SetInitialGuess(self.x_vars[1][i], init_guess2[i])
            self.prog.SetInitialGuess(self.x_vars[2][i], init_guess3[i])
            self.prog.SetInitialGuess(self.x_vars[3][i], init_guess4[i])
    
    def AddTimeInitialGuess(self, tf0):
        init_guess = np.linspace(0, tf0, self.N)
        for i in range(self.N-1):
            self.prog.SetInitialGuess(self.h_vars[i], init_guess[i+1]-init_guess[i])

    def AddFinalStateCost(self, QN):
        x_final = self.x_vars.T[-1]
        self.prog.AddCost(x_final.T.dot(QN.dot(x_final)))
    
    def AddTimePenalization(self, traj_vars, penalty):
        cost = 0
        for i in range(len(traj_vars)):
            cost = cost + traj_vars[i]
        self.prog.AddCost(penalty*cost)
    
    def AddEqualTimeStepsConstraint(self, traj_vars):
        for i in range(1,len(traj_vars)):
            self.prog.AddConstraint(traj_vars[i-1] == traj_vars[i])

    def running_cost(self,h,x,u):
        Q = h*np.eye(4)
        R = h*self.options["R"]
        g = .5*(x-self.options["xG"]).dot(Q.dot(x-self.options["xG"])) + .5*R*u**2
        return g

    def AddRunningCost(self, traj_vars):
        N = self.options["N"]

        x_vars = np.array([traj_vars[[i for i in range(N)]],traj_vars[[N+i for i in range(N)]],traj_vars[[2*N+i for i in range(N)]],traj_vars[[3*N+i for i in range(N)]]]).T
        u_vars = np.array(traj_vars[[(4*N + i) for i in range(N-1)]])
        h_vars = np.array(traj_vars[[(5*N -1 + i) for i in range(N-1)]])

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
            time_traj = np.append(time_traj, [t_prev + h_i])
            t_prev = t_prev + h_i
        state_traj = result.GetSolution(self.x_vars)
        input_traj = np.append([0],result.GetSolution(self.u_vars))

        return time_traj, state_traj, input_traj

    def dynamics_integration(self,x,u, h, integrator):
        if integrator == "EULER":
            f_n, df_n = self.sys.continuous_dynamics(x,u)
            x_nplus1 = np.array(x) + h*np.array(f_n)
            dx_nplus1dh = f_n
            dx_nplus1dx = np.eye(4) + h*np.array(df_n[1])
            dx_nplus1du = h*np.array(df_n[2])
            dx_nplus1dw = h*np.array(df_n[3])
            dx_nplus1 = [dx_nplus1dh,dx_nplus1dx,dx_nplus1du,dx_nplus1dw]
        elif integrator == "MIDPOINT":
            f_n, df_n = self.sys.continuous_dynamics(x,u)
            x_mid = np.array(x) + (h/2)*np.array(f_n)
            f_mid, df_mid = self.sys.continuous_dynamics(x_mid,u)
            x_nplus1 = np.array(x) + h*np.array(f_mid)
            dx_nplus1dh = f_mid
            dx_nplus1dx = np.eye(4) + h*np.array(df_mid[1])
            dx_nplus1du = h*np.array(df_mid[2])
            dx_nplus1dw = h*np.array(df_mid[3])
            dx_nplus1 = [dx_nplus1dh,dx_nplus1dx,dx_nplus1du,dx_nplus1dw]
              
        return x_nplus1, dx_nplus1