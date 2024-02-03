
########################
# Time invarying RoA est
########################

import numpy as np
from cart_pole.controllers.tvlqr.RoAest.utils import quad_form, sampleFromEllipsoid

class probTVROA:
    """
    class for probabilistic RoA estimation of linear time variant systems under (finite horizon) TVLQR control
    takes a configuration dict and a step simulation function. 

    the roaConf dict has the following structure:
    
        roaConf={nSimulations:<int>,nKnotPoints:<int>,rhof:<float>,rho00:<float>,S:<np.array>}

    the step function should be a callback in which a piecewise simulation of the closed loop dynamics is implemented.
    it takes as an input the knot point until which to simulate.
    it should return a short dict with a simulation result and the deviation from the nominal trajectory in error coordinates xBar
        
        e.g.:

            def step(knotPoint):

                <Propagate dynamics to timestep of next knotpoint. only consider x passed above when launching new sim >

                if simSuccess:
                    return True,xBark
                else
                    return False,xBark

    Additionally a callback function has to be implemented to start a new simulation:

        e.g.:

            def newSim(xBar):
                < prepare new simulation with x=x00+xBar >


    Would probably make sense to put all simulation related information into a class and have the step and newSim functions as member functions.
    
    Then the entire process of RoA estimation could look like this:

        1. create simulation class object. 
        2. initialize tvroa estimation routine. pass relevant information from simulation class (roaConf) and name of callback (step) here.
        3. Do Roa estimation. for model evaluation /simulation, call the callback function previously defined
    """

    def __init__(self,roaConf, simulator, verbose = False):
        self.nSimulations = roaConf["nSimulations"]   # number of simulations
        self.rho00 = roaConf["rho00"]                 # big initial guess for rho00
        self.rho_f = roaConf["rho_f"]                 # fixed size of rhof that determines the size of the last ellipsoid around the end of the trajectory
        self.dt = roaConf["dt_sim"] 
        self.simulator = simulator
        self.timeStark= self.simulator.T_nom          # nominal t's
        self.xStark= self.simulator.X_nom             # nominal x's
        self.S = self.simulator.tvlqr_S               # S matrix from the tvlqr controller
        self.nEvalPoints = len(self.timeStark)        # n of knot points
        self.verbose = verbose

        # array to store the evolution of rho defined by failed simulations
        self.rhoHist=np.ones(self.nEvalPoints)*self.rho00 # init of rhoHist and set initial guess
        self.rhoHist[-1]=self.rho_f                        # set final rho to const value

        # also store the cost to go evolution for those simulations that were successfull.
        self.ctgHist=np.ones(self.nEvalPoints)*np.inf 

    def doEstimate(self):

        for l in range(1,self.nEvalPoints):  # the trajectory has nKnotpoints-1 intervals or piecewise simulations
            k = (self.nEvalPoints-l-1)       # going backward, from the last interval to the first
            kPlus1 = (self.nEvalPoints-l)
            Sk = self.S.value(self.timeStark[k])
            SkPlus1 = self.S.value(self.timeStark[kPlus1])

            # Max successfull simulations
            self.maxSuccSimulations = self.nSimulations/2

            if self.verbose:
                    print(f"knot point {k}")
                          
            for j in range(self.nSimulations): 
                xBark = sampleFromEllipsoid(Sk,self.rhoHist[k])
                xk = xBark + self.xStark.T[k]
                
                self.ctgHist[k]= quad_form(Sk,xBark)
                termReason=0 # suppose a successful result for the simulation

                self.simulator.init_simulation(x0 = xk, init_knot = k, dt_sim = self.dt) #, final_knot = kPlus1) # simulation initialization
                T_sim, X_sim, U_sim =self.simulator.simulate() # simulation of the desired interval
                xkPlus1 = X_sim.T[-1]

                xBarkPlus1=xkPlus1-self.xStark.T[-1] #self.xStark.T[kPlus1]               
                self.ctgHist[kPlus1]= quad_form(SkPlus1,xBarkPlus1) # final cost to go calculation

                # is it inside the next ellipse?
                if self.ctgHist[kPlus1] > self.rhoHist[-1]: #self.rhoHist[kPlus1]: # no -> shrink
                    termReason=1
                    self.rhoHist[k] = min(self.ctgHist[k], self.rhoHist[k])
                else:
                    self.maxSuccSimulations = self.maxSuccSimulations-2

                if self.maxSuccSimulations == 0: # enough successes
                    termReason=3

                if self.rhoHist[k]<= 1e-06: # bad knot point
                    self.rhoHist[[s for s in range(k)]] = 1e-10
                    self.ctgHist[[s for s in range(k)]] = 1e+3
                    termReason=2

                # if self.verbose:
                #     print(f"simulation {j}, termination reason:"+str(termReason))
                
                if termReason > 1:
                    break
            
            if termReason == 2:
                    break
                
            if self.verbose:
                print("rhoHist :")
                print(self.rhoHist)
                print("---")

        return self.rhoHist, self.S