import numpy as np
from pydrake.all import FiniteHorizonLinearQuadraticRegulatorOptions, \
                        FiniteHorizonLinearQuadraticRegulator, \
                        PiecewisePolynomial, \
                        Linearize, \
                        LinearQuadraticRegulator, \
                        DiagramBuilder, \
                        AddMultibodyPlantSceneGraph, \
                        Parser

# FiniteHorizonLinearQuadraticRegulator(plant, context, t0, tf, Q, R)


class TVLQRController:
    """
    Controller acts on a predefined trajectory.
    """
    def __init__(self, data_dict, urdf_path, force_limit=9):
        """
        Parameters
        ----------
        data_dict : dictionary
            a dictionary containing the trajectory to follow
            should have the entries:
            data_dict["des_time_list"] : desired timesteps
            data_dict["des_cart_pos_list"] : desired cart positions
            data_dict["des_pend_pos_list"] : desired pendulum positions
            data_dict["des_cart_vel_list"] : desired cart velocities
            data_dict["des_pend_vel_list"] : desired pendulum velocities
            data_dict["des_force_list"] : desired forces
        urdf_path : path
            a path leading towards an urdf file describing the cartpole
        force_limit : float, default=np.inf
            the torque_limit of the pendulum actuator
        """

        # load the trajectory
        self.traj_time = data_dict["des_time_list"]
        self.traj_x1 = data_dict["des_cart_pos_list"]
        self.traj_x2 = data_dict["des_pend_pos_list"]
        self.traj_x3 = data_dict["des_cart_vel_list"]
        self.traj_x4 = data_dict["des_pend_vel_list"]
        self.traj_force = data_dict["des_force_list"]

        self.max_time = self.traj_time[-1]

        self.traj_time = np.reshape(self.traj_time, (self.traj_time.shape[0], -1))
        self.traj_force = np.reshape(self.traj_force, (self.traj_force.shape[0], -1)).T

        x0_desc = np.vstack((self.traj_x1, self.traj_x2, self.traj_x3, self.traj_x4))

        u0 = PiecewisePolynomial.FirstOrderHold(self.traj_time, self.traj_force)
        x0 = PiecewisePolynomial.CubicShapePreserving(self.traj_time, x0_desc, zero_end_point_derivatives=True)

        # create plant from urdf
        builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(builder, 0)
        Parser(self.plant).AddModelFromFile(urdf_path)
        self.plant.Finalize()

        self.force_limit = force_limit

        # create lqr context
        self.tilqr_context = self.plant.CreateDefaultContext()
        # self.plant.get_input_port(0).FixValue(self.tilqr_context, [0])

        self.input_i = self.plant.get_actuation_input_port().get_index()
        self.output_i = self.plant.get_state_output_port().get_index()
        self.plant.get_actuation_input_port().FixValue(self.tilqr_context, [0])
        # self.Q_tilqr = np.diag((1., 6., 0., 0.))
        # self.R_tilqr = np.array([0.0004])
        self.Q_tilqr = np.diag([10., 100., .1, .1])  
        self.R_tilqr = np.eye(1) * 1

        # Setup Options and Create TVLQR
        self.options = FiniteHorizonLinearQuadraticRegulatorOptions()
        self.options.input_port_index = self.input_i
        self.Q = np.diag([10., 100., .1, .1])
        self.options.u0 = u0
        self.options.x0 = x0

        self.counter = 0
        self.last_cart_pos = self.traj_x1[0]
        self.last_pend_pos = self.traj_x2[0]
        self.last_cart_vel = self.traj_x3[0]
        self.last_pend_vel = self.traj_x4[0]

    def set_goal(self, x):
        #  pos = (x[1] + np.pi) % (2 * np.pi) - np.pi
        # self.tilqr_context.SetContinuousState([0, np.pi, 0, 0])
        self.tilqr_context.SetContinuousState([x[0], x[1], x[2], x[3]])
        linearized_cartpole = Linearize(self.plant, self.tilqr_context, self.input_i, self.output_i,
                                        equilibrium_check_tolerance=1e-3)  # equilibrium_check_tolerance=1e-3
        (K, S) = LinearQuadraticRegulator(linearized_cartpole.A(), linearized_cartpole.B(), self.Q_tilqr, self.R_tilqr)
        self.options.Qf = S #np.diag([250, 1500, 0, 0])

        self.tvlqr = FiniteHorizonLinearQuadraticRegulator(
            self.plant,
            self.tilqr_context,
            t0=self.options.u0.start_time(),
            tf=self.options.u0.end_time(),
            Q=self.Q,
            R=np.eye(1),
            options=self.options)

    def get_control_output(self, mea_time, mea_cart_pos, mea_pend_pos, mea_cart_vel, mea_pend_vel):
        """
        The function to read and send the entries of the loaded trajectory
        as control input to the simulator/real pendulum.

        Parameters
        ----------
        mea_time : float, default=None
            the collapsed time [s]
        mea_cart_pos : float, default=None
            the position of the cart [m]
        mea_pend_pos : float, default=None
            the position of the pendulum [rad]
        mea_cart_vel : float, default=None
            the velocity of the cart [m/s]
        mea_pend_vel : float, default=None
            the velocity of the pendulum [rad/s]
        mea_force : float, default=None
            the measured force on the cart [N]

        Returns
        -------
        des_cart_pos : float
            the desired position of the cart [m]
        des_pend_pos : float
            the desired position of the pendulum [rad]
        des_cart_vel : float
            the desired velocity of the cart [m/s]
        des_pend_vel : float
            the desired velocity of the pendulum [rad/s]
        des_force : float
            the force supposed to be applied by the actuator [N]
        """

        x = np.array([[float(np.squeeze(mea_cart_pos))], [float(np.squeeze(mea_pend_pos))],
                      [float(np.squeeze(mea_cart_vel))], [float(np.squeeze(mea_pend_vel))]])

        # des_cart_pos = self.traj_x1[self.counter]
        # des_pend_pos = self.traj_x2[self.counter]
        # des_cart_vel = self.traj_x3[self.counter]
        # des_pend_vel = self.traj_x4[self.counter]

        # des_cart_pos = self.last_cart_pos
        # des_pend_pos = self.last_pend_pos
        # des_cart_vel = self.last_cart_vel
        # des_pend_vel = self.last_pend_vel
        # des_force = 0

        # if self.counter < len(self.traj_time):
        #     des_cart_pos = self.traj_x1[self.counter]
        #     des_pend_pos = self.traj_x2[self.counter]
        #     des_cart_vel = self.traj_x3[self.counter]
        #     des_pend_vel = self.traj_x4[self.counter]
        #     des_force = self.traj_force[self.counter]
        #     self.last_cart_pos = x[0]
        #     self.last_pend_pos = x[1]
        #     self.last_cart_vel = x[2]
        #     self.last_pend_vel = x[3]

        self.counter += 1
        time = min(mea_time, self.max_time)

        uu = self.tvlqr.u0.value(time)
        xx = self.tvlqr.x0.value(time)
        KK = self.tvlqr.K.value(time)
        kk = self.tvlqr.k0.value(time)

        xdiff = x - xx
        # xdiff[1] = (xdiff[1] + np.pi) % (2 * np.pi) - np.pi

        # des_force = (uu - KK.dot(xdiff) - kk)[0][0]
        des_force = (uu - KK.dot(xdiff))[0][0]
        # des_force = np.clip(des_force, -self.force_limit, self.force_limit)

        # des_cart_pos, des_pend_pos, des_cart_vel, des_pend_vel,
        return des_force
