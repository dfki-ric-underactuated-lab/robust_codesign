import numpy as np

from pydrake.systems.analysis import GetIntegrationSchemes, ResetIntegratorFromFlags
from pydrake.multibody.parsing import Parser
from pydrake.all import FiniteHorizonLinearQuadraticRegulatorOptions, \
                        FiniteHorizonLinearQuadraticRegulator, \
                        MakeFiniteHorizonLinearQuadraticRegulator, \
                        PiecewisePolynomial, \
                        Linearize, \
                        LinearQuadraticRegulator, \
                        DiagramBuilder, \
                        AddMultibodyPlantSceneGraph, MultibodyPlant, ConstantVectorSource, Adder, \
                        Parser, BasicVector, Saturation, LogVectorOutput, Simulator, ResetIntegratorFromFlags, SimulatorConfig, ApplySimulatorConfig

class DrakeStepSimulator():
    def __init__(self, urdf_path, data_dict, force_limit, dt_sim = 0.01):
        # Plant from urdf
        self.urdf = urdf_path

        # Trajectory from csv
        self.data_dict = data_dict
        self.T_nom = data_dict["des_time_list"]

        # Saturation and logging parameters
        self.force_limit = force_limit
        self.dt_log = dt_sim

        # Initial state
        self.x0 = [ data_dict["des_cart_pos_list"][0], data_dict["des_pend_pos_list"][0],
                    data_dict["des_cart_vel_list"][0], data_dict["des_pend_vel_list"][0]]

        # simulation time step
        self.dt_sim = dt_sim

        # Setup plant
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
        Parser(plant).AddModelFromFile(self.urdf)
        plant.Finalize()
        context = plant.CreateDefaultContext()
     
        # LQR controller for last K and S
        xG = np.array([0,0,0,0])
        tilqr_context = plant.CreateDefaultContext()
        input_i = plant.get_actuation_input_port().get_index()
        output_i = plant.get_state_output_port().get_index()
        plant.get_actuation_input_port().FixValue(tilqr_context, [0])
        # Q_tilqr = np.diag((1., 10., .1, .1))
        # R_tilqr = np.array([.0005])
        Q_tilqr = np.diag((10., 100., .1, .1))
        R_tilqr = np.array([1])
        tilqr_context.SetContinuousState([xG[0], xG[1], xG[2], xG[3]])
        linearized_cartpole = Linearize(plant, tilqr_context, input_i, output_i,
                                            equilibrium_check_tolerance=1e-3)  # equilibrium_check_tolerance=1e-3
        (K, S) = LinearQuadraticRegulator(linearized_cartpole.A(), linearized_cartpole.B(), Q_tilqr, R_tilqr)

        # Setup tvlqr controller   
        traj_time = self.data_dict["des_time_list"]
        traj_x1 = self.data_dict["des_cart_pos_list"]
        traj_x2 = self.data_dict["des_pend_pos_list"]
        traj_x3 = self.data_dict["des_cart_vel_list"]
        traj_x4 = self.data_dict["des_pend_vel_list"]
        traj_force = self.data_dict["des_force_list"]
        traj_time = np.reshape(traj_time, (traj_time.shape[0], -1))
        traj_force = np.reshape(traj_force, (traj_force.shape[0], -1)).T
        x0_desc = np.vstack((traj_x1, traj_x2, traj_x3, traj_x4))
        self.X_nom = x0_desc
        u0 = PiecewisePolynomial.FirstOrderHold(traj_time, traj_force)
        x0 = PiecewisePolynomial.CubicShapePreserving(traj_time, x0_desc, zero_end_point_derivatives=True)
        options = FiniteHorizonLinearQuadraticRegulatorOptions()
        options.input_port_index = input_i
        # Q = np.diag([1., 100., 1., 10.])  
        # R = np.eye(1) * .1
        Q = np.diag((100., 100., 1, .1))
        R = np.array([1])
        options.u0 = u0
        options.x0 = x0
        options.Qf = S
        controller = FiniteHorizonLinearQuadraticRegulator(
                plant,
                context,
                t0= options.u0.start_time(),
                tf= options.u0.end_time(),
                Q= Q,
                R= R,
                options= options)
        self.tvlqr_S = controller.S
        controller_sys = MakeFiniteHorizonLinearQuadraticRegulator(
                plant,
                context,
                t0= options.u0.start_time(),
                tf= options.u0.end_time(),
                Q= Q,
                R= R,
                options= options)
        controller_plant = builder.AddSystem(controller_sys)

        # Setup saturation block
        saturation = builder.AddSystem(Saturation(min_value=[-self.force_limit], max_value=[self.force_limit]))

        # Add blocks connections
        builder.Connect(controller_plant.get_output_port(),
                        saturation.get_input_port())
        builder.Connect(saturation.get_output_port(),
                        plant.get_actuation_input_port())
        builder.Connect(plant.get_state_output_port(),
                        controller_plant.get_input_port()) 
        
        # Setup a logger for the acrobot state
        self.state_logger = LogVectorOutput(plant.get_state_output_port(), builder, self.dt_log)
        self.input_logger = LogVectorOutput(saturation.get_output_port(), builder, self.dt_log)

        # Build-up the diagram
        self.diagram = builder.Build()

    def init_simulation(self):
        # Set up a simulator to run this diagram
        self.simulator = Simulator(self.diagram)

        # Simulator configuration
        #print(GetIntegrationSchemes()): ['bogacki_shampine3', 'explicit_euler', 'implicit_euler', 'radau1', 'radau3', 'runge_kutta2', 'runge_kutta3', 'runge_kutta5', 'semi_explicit_euler', 'velocity_implicit_euler']
        config = SimulatorConfig()
        config.max_step_size = self.dt_sim
        config.target_realtime_rate = 0
        config.publish_every_time_step = True
        config.integration_scheme = 'explicit_euler'
        ApplySimulatorConfig(config, self.simulator)

    def simulate(self, x0 = None, init_knot = 0, final_knot = -1):
        # Define timings and states for the simulation
        t0 = self.T_nom[init_knot]
        tf = self.T_nom[final_knot]

        # Set the initial conditions (theta1, theta2, theta1dot, theta2dot)
        context = self.simulator.get_mutable_context()
        if x0 is None:
            x0 = self.x0
        context.SetContinuousState(x0)
        context.SetTime(t0)

        # Simulate
        self.simulator.AdvanceTo(tf)

        # Collect the resulting trajectories
        x_sim = self.state_logger.FindLog(context).data()
        u_sim = self.input_logger.FindLog(context).data()
        t_sim = np.linspace(t0,tf,len(x_sim.T))

        return t_sim, x_sim, u_sim

class openloop_DrakeStepSimulator():
    def __init__(self, urdf_path, force_limit, dt_sim = 0.01):
        # Plant from urdf
        self.urdf = urdf_path

        # Saturation and logging parameters
        self.force_limit = force_limit
        self.dt_log = dt_sim

        # Initial state
        self.x0 = [0.001,0.001,0.001,0.001]

        # sim ulation time step
        self.dt_sim = dt_sim

        # Setup plant
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
        Parser(plant).AddModelFromFile(self.urdf)
        plant.Finalize()
        context = plant.CreateDefaultContext()
        
        # Zero input
        controller_plant = builder.AddSystem(ConstantVectorSource([0]))

        # Add blocks connections
        builder.Connect(controller_plant.get_output_port(),
                        plant.get_actuation_input_port())
        
        # Setup a logger for the acrobot state
        self.state_logger = LogVectorOutput(plant.get_state_output_port(), builder, self.dt_log)

        # Build-up the diagram
        self.diagram = builder.Build()

    def simulate(self, x0, tf):
        # Define timings and states for the simulation
        t0 = 0

        # Set up a simulator to run this diagram
        self.simulator = Simulator(self.diagram)
        
        # Simulator configuration
        #print(GetIntegrationSchemes()): ['bogacki_shampine3', 'explicit_euler', 'implicit_euler', 'radau1', 'radau3', 'runge_kutta2', 'runge_kutta3', 'runge_kutta5', 'semi_explicit_euler', 'velocity_implicit_euler']
        config = SimulatorConfig()
        config.max_step_size = self.dt_sim
        config.target_realtime_rate = 0
        config.publish_every_time_step = True
        config.integration_scheme = 'explicit_euler'
        ApplySimulatorConfig(config, self.simulator)

        # Set the initial conditions (theta1, theta2, theta1dot, theta2dot)
        context = self.simulator.get_mutable_context()
        context.SetContinuousState(x0)
        context.SetTime(t0)

        # Simulate
        self.simulator.AdvanceTo(tf)

        # Collect the resulting trajectories
        x_sim = self.state_logger.FindLog(context).data()
        u_sim = np.zeros((len(x_sim.T),1))
        t_sim = np.linspace(t0,tf,len(x_sim.T))

        return t_sim, x_sim.T, u_sim

class StepSimulator():
    def __init__(self, cartpole, controller_options, verbose = False):  
        self.ss = False # State saturation flag
        self.verboseSim = verbose # Verbosity
        self.sys = cartpole["sys"]
        self.force_limit = self.sys.fl # input saturation
        self.x_lim = cartpole["x_lim"]

        # Trajectory from csv
        self.X_nom = controller_options["X_nom"]
        self.U_nom = controller_options["U_nom"]
        self.T_nom = controller_options["T_nom"]

        # Drake TVLQR init
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
        Parser(plant).AddModelFromFile(cartpole["urdf"])
        plant.Finalize()
        context = plant.CreateDefaultContext()
        xG = controller_options["xG"]
        tilqr_context = plant.CreateDefaultContext()
        input_i = plant.get_actuation_input_port().get_index()
        output_i = plant.get_state_output_port().get_index()
        plant.get_actuation_input_port().FixValue(tilqr_context, [0])
        tilqr_context.SetContinuousState([xG[0], xG[1], xG[2], xG[3]])
        linearized_cartpole = Linearize(plant, tilqr_context, input_i, output_i,
                                            equilibrium_check_tolerance=1e-3)  # equilibrium_check_tolerance=1e-3
        (K, S) = LinearQuadraticRegulator(linearized_cartpole.A(), linearized_cartpole.B(), controller_options["Q"], controller_options["R"])  
        traj_time = np.reshape(self.T_nom, (self.T_nom.shape[0], -1))
        traj_force = np.reshape(self.U_nom, (self.U_nom.shape[0], -1)).T
        u0 = PiecewisePolynomial.FirstOrderHold(traj_time, traj_force)
        x0 = PiecewisePolynomial.CubicShapePreserving(traj_time, self.X_nom, zero_end_point_derivatives=True)
        options = FiniteHorizonLinearQuadraticRegulatorOptions()
        options.input_port_index = input_i
        options.u0 = u0
        options.x0 = x0
        options.Qf = S
        self.controller = FiniteHorizonLinearQuadraticRegulator(
                plant,
                context,
                t0= options.u0.start_time(),
                tf= options.u0.end_time(),
                Q= controller_options["Q"],
                R= controller_options["R"],
                options= options)
        self.tvlqr_S = self.controller.S
    
    def init_simulation(self, x0 = None, init_knot = 0, final_knot = -1, dt_sim = 0.01):
        # Initialize simulated trajectory
        self.dt_sim = dt_sim # simulation frequency
        self.N_sim = int((self.T_nom[final_knot]-self.T_nom[init_knot])/self.dt_sim)
        # self.T_sim, self.dt_sim = np.linspace(self.T_nom[init_knot],self.T_nom[final_knot], self.N_sim, retstep=True)
        self.T_sim = np.zeros((self.N_sim,1))
        self.T_sim[0] = self.T_nom[init_knot]
        self.X_sim = np.zeros((len(self.T_sim),4))
        if x0 is None:
            self.X_sim[0] = self.X_nom.T[init_knot]
        else:
            self.X_sim[0] = x0
        self.U_sim = np.zeros((len(self.T_sim),1))
        self.U_sim[0] = self.U_nom[init_knot]

        self.init_knot = init_knot
        self.final_knot = final_knot
        if final_knot == -1:
            self.final_knot = len(self.T_nom)-1

    def simulate(self, integrator = "euler"):
        
        for i in range(self.N_sim-1):
            x_star = self.controller.x0.value(self.T_sim[i]).T[0]
            u_star = self.controller.u0.value(self.T_sim[i])[0][0]
            self.U_sim[i+1] = np.clip(u_star - self.controller.K.value(self.T_sim[i]).dot((self.X_sim[i]-x_star)),-self.force_limit,self.force_limit)

            # Dynamics integration
            self.X_sim[i+1] = self.integration(integrator,self.sys,self.dt_sim,self.X_sim[i],self.U_sim[i+1], self.T_sim[i])
            self.T_sim[i+1] = self.T_sim[i] + self.dt_sim

            # State saturation warning
            if self.X_sim[i+1][0]<= -self.x_lim or self.X_sim[i+1][0]>= self.x_lim:
                self.ss = True
        if self.ss and self.verboseSim:
            print("State saturation warning")
            
        return self.T_sim, self.X_sim.T, self.U_sim.T
    
    def integration(self,type, sys, h, x_i, u_iplus1, t_i):
        if type == "euler":
            f,df = sys.continuous_dynamics(x_i, u_iplus1)
            x_iplus1 = x_i + h*f
        if type == "rk4":
            K1 = h*sys.continuous_dynamics(x_i, u_iplus1)
            K2 = h*sys.continuous_dynamics(x_i + K1/2, u_iplus1)
            K3 = h*sys.continuous_dynamics(x_i + K2/2, u_iplus1)
            K4 = h*sys.continuous_dynamics(x_i + K3, u_iplus1)
            x_iplus1 = x_i + K1/6 + K2/3 + + K3/3 + K4/6
        return x_iplus1