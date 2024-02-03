import numpy as np
import os
from quanser.hardware import HIL, HILError, EncoderQuadratureMode
import time
import matplotlib.pyplot as plt
from model.parameters import Cartpole
from controllers.lqr.lqr import lqr

# desired
state_desired = np.array([[0], [0], [0], [0]])  # [[mm], [rad], [mm/s], [rad/s]] desired state for lqr
limit = 0.3  # [m] maximum deviation from initial cart position
V_max = 4  # [V] maximum voltage on motor
eta = np.radians(15)  # [rad] lqr engagement bounds
k_ec = -10.0  # [-] input constant for energy control
runtime = 25  # [s] time inside control loop

# trajectory
csv_path = os.path.join("log_data", "trajectory_sin.csv")
trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
u = trajectory.T[5].T  # input array
N = u.shape[0]
dt = trajectory.T[0].T[1]
control_frequency = 1/dt  # [Hz] constant control frequency
runtime = N * dt

# system setup
selection = "short"  # pendulum length: "short" or "long"
sys = Cartpole(selection)  # setup cartpole system
pos_cart_factor = 1.66 * 56 / (4096 * 1000)  # cart position factor for conversion to meters
pos_pend_factor = 2 * np.pi / 4096  # pendulum position factor for conversion to radians
# K = np.array([[-50, 160.72, -47.46, 14.86]])

# initialization
k_loop = 0  # loop counter
# time setup
time_start = 0.0
time_start_last_iteration = 0.0
time_loop_max = 0.0  # [s] initial value maximum time for one iteration
time_loop_min = 1.0  # [s] initial value minimum time for one iteration
# recording setup
time_from_last_rec = 0.0
switch = 0  # switch variable for simple if statements
samples_max = N + 1  # maximum samples
t = np.zeros(samples_max)  # time array
rec_command_out = np.zeros(samples_max)  # desired command array
rec_command_in = np.zeros(samples_max)  # received command array
force = np.zeros(samples_max)  # measured force
state = np.zeros((4, samples_max))  # state matrix
# card setup
samples_in_buffer = 1000
channels = np.array([0, 1], dtype=np.uint32)  # corresponds to adc out
num_channels = len(channels)
buffer_in = np.zeros(num_channels, dtype=np.float64)
enc_buffer_in = np.zeros(num_channels, dtype=np.int32)
buffer_out = np.array([0.0, 0.0], dtype=np.float64)

try:
    # communication
    card = HIL("q2_usb", "0")
    card.task_create_analog_reader(samples_in_buffer, channels, num_channels)
    card.set_encoder_counts(channels, num_channels, enc_buffer_in)
    card.set_encoder_quadrature_mode(channels, num_channels, np.array([EncoderQuadratureMode.X4,
                                                                       EncoderQuadratureMode.X4], dtype=np.uint32))
    print('starting control')
    # initial recordings
    time_0 = time.time()

    while time_start < runtime-dt:
        time_start = time.time() - time_0

        # read
        card.read(channels, num_channels, channels, num_channels,
                  None, 0, None, 0, buffer_in, enc_buffer_in, None, None)

        # create state vector
        state[0, k_loop] = enc_buffer_in[0] * pos_cart_factor
        state[1, k_loop] = enc_buffer_in[1] * pos_pend_factor + np.pi
        # state[1, k_loop] = (state[1, k_loop] + np.pi) % (2 * np.pi) - np.pi
        # state[1, k_loop] = np.remainder(enc_buffer_in[1] * pos_pend_factor + np.pi, 2 * np.pi)
        # if state[1, k_loop] > np.pi:
        #     state[1, k_loop] = state[1, k_loop] - 2 * np.pi
        if k_loop > 0:
            state[2, k_loop] = (state[0, k_loop] - state[0, k_loop - 1]) / (time_start - time_start_last_iteration)
            state[3, k_loop] = (state[1, k_loop] - state[1, k_loop - 1]) / (time_start - time_start_last_iteration)
            # if np.abs(state[1, k_loop] - state[1, k_loop - 1]) > np.pi:
            #     state[3, k_loop] = (state[1, k_loop] - state[1, k_loop - 1] - 2 * np.pi *
            #                         (2 * (state[1, k_loop] >= 0) - 1)) / (time_start - time_start_last_iteration)

        # cart command
        buffer_out[0] = sys.amplitude(u[k_loop], state[2, k_loop])

        # displacement constraints
        if state[0, k_loop] < -limit or state[0, k_loop] > limit:
            if np.sign(buffer_out[0]) == np.sign(state[0, k_loop]):
                buffer_out = np.array([0.0, 0.0], dtype=np.float64)

        # voltage constraints
        buffer_out[0] = np.clip(buffer_out[0], -V_max, V_max)

        # write
        # print(f'Output: {buffer_out}')
        card.write_analog(channels, num_channels, buffer_out)

        # measurement: control frequency
        if k_loop > 0:
            if (time_start - time_start_last_iteration) > time_loop_max:
                time_loop_max = time_start - time_start_last_iteration

            if (time_start - time_start_last_iteration) < time_loop_min:
                time_loop_min = time_start - time_start_last_iteration

        # recording
        t[k_loop] = time_start
        rec_command_out[k_loop] = buffer_out[0]
        rec_command_in[k_loop] = buffer_in[0]
        force[k_loop] = sys.force(buffer_out[0], state[2, k_loop])

        # loop variables
        k_loop += 1
        time_start_last_iteration = time_start

        # force constant control frequency
        while time.time() - time_start - time_0 < dt:
            pass

    # security
    buffer_out = np.array([0.0, 0.0], dtype=np.float64)
    card.write_analog(channels, num_channels, buffer_out)

    # output: control frequency measurement
    mean_frequ = k_loop / (time.time() - time_0)
    print(f'Mean control frequency: {mean_frequ} Hz')
    print(f'Minimum control frequency: {1 / time_loop_max} Hz')
    print(f'Maximum control frequency: {1 / time_loop_min} Hz')

    # output plots
    plt.figure(figsize=(12, 6))
    plt.subplot(2, 1, 1)
    plt.plot(t[:k_loop], rec_command_out[:k_loop], label='amplifier command')
    plt.ylabel('Amplifier Command [V]')
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.plot(t[:k_loop], rec_command_in[:k_loop], label='current sense')
    plt.ylabel('Current Sense [V/A]')
    plt.legend()
    plt.show()

    plt.figure(figsize=(12, 6))
    plt.subplot(5, 1, 1)
    plt.plot(t[:k_loop], trajectory.T[1].T * 1000, label='des')
    plt.plot(t[:k_loop], state[0, :k_loop] * 1000, label='mea')
    plt.ylabel('Cart Position [mm]')
    plt.legend()
    plt.subplot(5, 1, 2)
    pendulum_position_desired = (trajectory.T[2].T + np.pi) % (2 * np.pi) - np.pi
    pendulum_position_measured = (state[1, :k_loop] + np.pi) % (2 * np.pi) - np.pi
    plt.plot(t[:k_loop], pendulum_position_desired, label='des')
    plt.plot(t[:k_loop], pendulum_position_measured, label='mea')
    plt.ylabel('Pendulum Position [rad]')
    plt.xlabel('Time [sec]')
    plt.legend()
    plt.subplot(5, 1, 3)
    plt.plot(t[:k_loop], trajectory.T[3].T * 1000, label='des')
    plt.plot(t[:k_loop], state[2, :k_loop] * 1000, label='mea')
    plt.ylabel('Cart Velocity [mm/s]')
    plt.xlabel('Time [sec]')
    plt.legend()
    plt.subplot(5, 1, 4)
    plt.plot(t[:k_loop], trajectory.T[4].T, label='des')
    plt.plot(t[:k_loop], state[3, :k_loop], label='mea')
    plt.ylabel('Pendulum Velocity [rad/s]')
    plt.xlabel('Time [sec]')
    plt.legend()
    plt.subplot(5, 1, 5)
    plt.plot(t[:k_loop], trajectory.T[5].T, label='des')
    plt.plot(t[:k_loop], force[:k_loop], label='mea')
    plt.ylabel('u | Cart Force [N]')
    plt.xlabel('Time [sec]')
    plt.legend()
    plt.show()

    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    ax.plot(pendulum_position_desired, t[:k_loop])
    ax.plot(pendulum_position_measured, t[:k_loop], label='des')
    ax.set_rmax(runtime)
    # ax.set_rticks([5, 10, 15, 20])  # Less radial ticks
    ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
    ax.set_theta_offset(np.pi / 2.0)
    ax.grid(True)
    ax.set_title("Pendulum Position", va='bottom')
    plt.show()

except HILError as e:
    print(e.get_error_message())
except KeyboardInterrupt:
    print("Aborted")
finally:
    buffer_out = np.array([0.0, 0.0], dtype=np.float64)
    card.write_analog(channels, num_channels, buffer_out)
    if card.is_valid():
        card.close()

print('all done, closing')
