import numpy as np
import math

from pydrake.solvers.snopt import SnoptSolver
from pydrake.all import MathematicalProgram, SolverOptions
from pydrake.autodiffutils import ExtractValue

class RobustDirtranTrajectoryOptimization():

    def __init__(self, sys, options, integrator = "MIDPOINT"):
        self.prog = MathematicalProgram()
        self.options = options
        self.sys = sys
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
        self.AddDynamicConstraints(self.options["x0"],self.options["xG"])

        # Add a linear constraint to a variable in all the knot points
        self.AddConstraintToAllKnotPoints(self.h_vars,self.options["hBounds"][0],self.options["hBounds"][1])
        self.AddConstraintToAllKnotPoints(self.u_vars,-self.options["fl"],self.options["fl"])
        self.AddConstraintToAllKnotPoints(self.x_vars[0],-self.options["cart_pos_lim"],self.options["cart_pos_lim"])

        # Add an initial guess for the resulting trajectory
        self.AddStateInitialGuess(self.options["x0"],self.options["xG"])
        self.AddTimeInitialGuess(self.options["tf0"])

        # Add cost on the final state
        #self.AddFinalStateCost(self.options["QN"])

        # Add constraints on the time trajectory
        #self.AddTimePenalization(self.h_vars, self.options["time_penalization"])
        self.AddEqualTimeStepsConstraint(self.h_vars)

        # Add running cost
        self.AddRunningCost(np.append(np.append(self.x_vars,self.u_vars), self.h_vars))

        # Add robust cost for dirtrel
        running_input_lowerBound = np.ones((self.options["N"]-2,1))*-self.options["fl"]
        running_input_upperBound = np.ones((self.options["N"]-2,1))*self.options["fl"]
        self.prog.AddConstraint(self.robust_input_constr_plus, running_input_lowerBound, running_input_upperBound, np.append(np.append(self.x_vars,self.u_vars), self.h_vars))
        self.prog.AddConstraint(self.robust_input_constr_minus, running_input_lowerBound, running_input_upperBound, np.append(np.append(self.x_vars,self.u_vars), self.h_vars))
        # running_state_lowerBound = np.ones((4*self.options["N"],1))*-np.inf
        # running_state_lowerBound[[i for i in range(self.options["N"])]] = np.ones((self.options["N"],1))*-self.options["cart_pos_lim"]
        # running_state_upperBound = np.ones((4*self.options["N"],1))*np.inf
        # running_state_upperBound[[i for i in range(self.options["N"])]] = np.ones((self.options["N"],1))*self.options["cart_pos_lim"]
        # self.prog.AddConstraint(self.robust_state_constr_plus, running_state_lowerBound, running_state_upperBound, np.append(self.x_vars,self.u_vars))
        # self.prog.AddConstraint(self.robust_state_constr_minus, running_state_lowerBound, running_state_upperBound, np.append(self.x_vars,self.u_vars))
        self.prog.AddCost(self.robust_cost,np.append(np.append(self.x_vars,self.u_vars), self.h_vars), description = "robust_cost")

        # Solve the Mathematical program
        solver = SnoptSolver()
        solver_options = SolverOptions()
        solver_options.SetOption(solver.id(),'Major Optimality Tolerance', 1e-3)
        solver_options.SetOption(solver.id(),'Major Feasibility Tolerance', 1e-4)
        solver_options.SetOption(solver.id(),'Minor Feasibility Tolerance', 1e-4)
        self.prog.SetSolverOptions(solver_options)

        #print('Solver Engaged')
        self.result = solver.Solve(self.prog)
        assert self.result.is_success()
        times, states, inputs = self.GetResultingTrajectory(self.result)
        return times, states, inputs 

    def AddDynamicConstraints(self, x0, xG):    
        self.prog.AddConstraint(self.x_vars[0][0] == x0[0])
        self.prog.AddConstraint(self.x_vars[1][0] == x0[1])
        self.prog.AddConstraint(self.x_vars[2][0] == x0[2])
        self.prog.AddConstraint(self.x_vars[3][0] == x0[3])
        for i in range(self.N-1):
            x_n = [self.x_vars[0][i],self.x_vars[1][i], self.x_vars[2][i], self.x_vars[3][i]]
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
        init_guess = tf0/(self.N-1)
        for i in range(self.N-1):
            self.prog.SetInitialGuess(self.h_vars[i], init_guess)

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
        Q = h*self.options["Q"]
        R = h*self.options["R"]
        g = .5*(x-self.options["xG"]).dot(Q.dot(x-self.options["xG"])) + .5*R*u**2
        dg = [g, (x-self.options["xG"]).dot(Q), R*u**2]
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
            time_traj = np.append(time_traj,[t_prev + h_i])
            t_prev = t_prev + h_i
        state_traj = result.GetSolution(self.x_vars)
        input_traj = np.append([0],result.GetSolution(self.u_vars))

        return time_traj,state_traj,input_traj 

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
    
    def robust_cost(self, traj_vars):
        N = self.options["N"]
        Rl = self.options["Rl"]
        Ql = self.options["Ql"]
        QNl = self.options["QNl"]
        
        K, self.E = self.deltaLQR(traj_vars)
            
        # Cost computation
        c = 0
        for k in range(N-1):
            c = c + np.trace((Ql + (K[:,:,k].T*Rl).dot(K[:,:,k])).dot(self.E[:,:,k]))
        c = c + np.trace(QNl.dot(self.E[:,:,N-1]))
        return c

    def robust_input_constr_plus(self, traj_vars):
        N = self.options["N"]
        
        K, E = self.deltaLQR(traj_vars)
            
        # Constraint computation
        v = np.zeros((N-2,), dtype = 'O')
        for k in range(1,N-1):
            U = K[:,:,k].dot(E[:,:,k].dot(K[:,:,k].T))
            Us = self.fastsqrt(U)
            v[k-1] = Us[0][0] 
        robust_input_vars = (np.array(traj_vars[[4*N+1 + i for i in range(N-2)]]) + v)
        return robust_input_vars
    
    def robust_input_constr_minus(self, traj_vars):
        N = self.options["N"]
        
        K, E = self.deltaLQR(traj_vars)
            
        # Constraint computation
        v = np.zeros((N-2,), dtype = 'O')
        for k in range(1,N-1):
            U = K[:,:,k].dot(E[:,:,k].dot(K[:,:,k].T))
            Us = self.fastsqrt(U)
            v[k-1] = Us[0][0]
        robust_input_vars = (np.array(traj_vars[[4*N+1 + i for i in range(N-2)]]) -v)
        return robust_input_vars
    
    def robust_state_constr_plus(self, traj_vars):
        N = self.options["N"]
        
        _,E = self.deltaLQR(traj_vars)

        # Constraint computation
        v = np.zeros(((4*N),), dtype = 'O')
        for k in range(1,4*N):
            if k<N:
                Pc = np.array([1,0,0,0])
                E_k = E[:,:,k]
            elif k>=N and k<2*N:
                Pc = np.array([0,1,0,0])
                E_k = E[:,:,k-N]
            elif k>=N and k<3*N:
                Pc = np.array([0,0,1,0])
                E_k = E[:,:,k-2*N]
            else:
                Pc = np.array([0,0,0,1])
                E_k = E[:,:,k-3*N]
            U = Pc.dot(E_k.dot(Pc))
            Us = self.fastsqrt([U])[0][0]
            if not math.isnan(Us):
                v[k] = Us
        robust_state_vars = (np.array(traj_vars[[i for i in range(4*N)]])+v)
        return robust_state_vars
    
    def robust_state_constr_minus(self, traj_vars):
        N = self.options["N"]
        
        _,E = self.deltaLQR(traj_vars)

        # Constraint computation
        v = np.zeros(((4*N),), dtype = 'O')
        for k in range(1,4*N):
            if k<N:
                Pc = np.array([1,0,0,0])
                E_k = E[:,:,k]
            elif k>=N and k<2*N:
                Pc = np.array([0,1,0,0])
                E_k = E[:,:,k-N]
            elif k>=N and k<3*N:
                Pc = np.array([0,0,1,0])
                E_k = E[:,:,k-2*N]
            else:
                Pc = np.array([0,0,0,1])
                E_k = E[:,:,k-3*N]
            U = Pc.dot(E_k.dot(Pc))
            Us = self.fastsqrt([U])[0][0]
            if not math.isnan(Us):
                v[k] = Us
        robust_state_vars = (np.array(traj_vars[[i for i in range(4*N)]])-v)
        return robust_state_vars
    
    def robust_dynamics(self, h, x,u, w):
        f_n, df_n = self.sys.continuous_dynamics_w(x,[u],w)
        x_nplus1 = np.array(x) + h*np.array(f_n)

        dx_nplus1dh = f_n
        dx_nplus1dx = np.eye(4) + h*np.array(df_n[1])
        dx_nplus1du = h*np.array(df_n[2])
        dx_nplus1dw = h*np.array(df_n[3])
        dx_nplus1 = [dx_nplus1dh,dx_nplus1dx,dx_nplus1du,dx_nplus1dw]

        return x_nplus1, dx_nplus1

    def deltaLQR(self, traj_vars):
        N = self.options["N"]
        D = self.options["D"]
        E1 = self.options["E1"]
        Q = self.options["Q"]
        QN = self.options["QN"]
        R = self.options["R"]

        x_vars = np.array([traj_vars[[i for i in range(N)]],traj_vars[[N+i for i in range(N)]],traj_vars[[2*N+i for i in range(N)]],traj_vars[[3*N+i for i in range(N)]]])
        u_vars = np.array(traj_vars[[(4*N + i) for i in range(N-1)]])
        h_vars = np.array(traj_vars[[(5*N -1 + i) for i in range(N-1)]])

        # Dynamics linearization
        A = np.zeros((4,4,N-1), dtype='O')
        B = np.zeros((4,1,N-1), dtype='O')
        G = np.zeros((4,1,N-1), dtype='O')
        for k in range(N-1):
            x_k = [x_vars[0][k],x_vars[1][k],x_vars[2][k],x_vars[3][k]]
            u_k = u_vars[k]
            h_k = h_vars[k]
            if self.integrator == "EULER":
                f_k, df_k = self.robust_dynamics(h_k, x_k, u_k, 0)
            elif self.integrator == "MIDPOINT":
                x_kplus1 = [x_vars[0][k+1],x_vars[1][k+1],x_vars[2][k+1],x_vars[3][k+1]]
                f_k, df_k = self.robust_dynamics(h_k, 0.5*(np.array(x_k)+np.array(x_kplus1)), u_k, 0)
            A[:,:,k] = np.array(df_k[1])
            B[:,:,k] = np.array([df_k[2]]).T
            G[:,:,k] = np.array([df_k[3]]).T

        # K tvlqr computation
        P = QN
        K = np.zeros((1,4,N-1), dtype='O')
        for k in np.flip(range(N-1)):
            K[:,:,k] = (B[:,:,k].T.dot(P.dot(A[:,:,k])))/(B[:,:,k].T.dot(P.dot(B[:,:,k]))+R)            
            P = Q + (K[:,:,k].T*R).dot(K[:,:,k]) + (A[:,:,k] - B[:,:,k].dot(K[:,:,k])).T.dot(P.dot(A[:,:,k] - B[:,:,k].dot(K[:,:,k])))

        # E and H calculation
        E = np.zeros((4,4,N), dtype='O')
        E[:,:,0] = E1
        H = np.zeros((4,1), dtype='O')
        for k in range(N-1):
            E[:,:,k+1] = (A[:,:,k] - B[:,:,k].dot(K[:,:,k])).dot(E[:,:,k].dot((A[:,:,k] - B[:,:,k].dot(K[:,:,k])).T)) + (A[:,:,k] - B[:,:,k].dot(K[:,:,k])).dot(H.dot(G[:,:,k].T)) + G[:,:,k].dot(H.T.dot((A[:,:,k] - B[:,:,k].dot(K[:,:,k])).T)) + G[:,:,k].dot(D*(G[:,:,k].T))
            H = (A[:,:,k] - B[:,:,k].dot(K[:,:,k])).dot(H) + G[:,:,k]*D

        return K, E
    
    def fastsqrt(self, A):
        '''FASTSQRT computes the square root of a matrix A with Denman-Beavers iteration'''
        A = ExtractValue(A)
        Ep = 1e-8*np.eye(A.shape[1])
        
        if np.count_nonzero(np.diag(A) > 0) != A.shape[1]:
            S = np.diag(np.sqrt(np.diag(A)))
            return S
        
        I = np.eye(A.shape[1])
        S = A
        T = I
        
        T = .5*(T + np.linalg.inv(S+Ep))
        S = .5*(S+I)
        for k in range(4):
            Snew = .5*(S + np.linalg.inv(T+Ep))
            T = .5*(T + np.linalg.inv(S+Ep))
            S = Snew

        return S

    def getOptimalCost(self):
        return self.result.get_optimal_cost()