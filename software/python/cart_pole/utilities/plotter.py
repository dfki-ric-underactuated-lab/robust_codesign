import numpy as np
import matplotlib.pyplot as plt


class Plotter:
    def __init__(self, data_dict):
        self.data = data_dict

    def states_and_input(self):
        plt.figure(figsize=(15, 10))
        plt.subplot(5, 1, 1)
        if self.data["des_force_list"].any():
            plt.plot(self.data["des_time_list"], self.data["des_cart_pos_list"] * 1000, label='des')
        if self.data["mea_force_list"].any():
            plt.plot(self.data["mea_time_list"], self.data["mea_cart_pos_list"] * 1000, label='mea')
        plt.ylabel('Cart Position [mm]')
        plt.legend()
        plt.subplot(5, 1, 2)
        if self.data["des_force_list"].any():
            plt.plot(self.data["des_time_list"], self.data["des_pend_pos_list"], label='des')
        if self.data["mea_force_list"].any():
            plt.plot(self.data["mea_time_list"], self.data["mea_pend_pos_list"], label='mea')
        plt.ylabel('Pendulum Position [rad]')
        plt.xlabel('Time [sec]')
        plt.legend()
        plt.subplot(5, 1, 3)
        if self.data["des_force_list"].any():
            plt.plot(self.data["des_time_list"], self.data["des_cart_vel_list"] * 1000, label='des')
        if self.data["mea_force_list"].any():
            plt.plot(self.data["mea_time_list"], self.data["mea_cart_vel_list"] * 1000, label='mea')
        plt.ylabel('Cart Velocity [mm/s]')
        plt.xlabel('Time [sec]')
        plt.legend()
        plt.subplot(5, 1, 4)
        if self.data["des_force_list"].any():
            plt.plot(self.data["des_time_list"], self.data["des_pend_vel_list"], label='des')
        if self.data["mea_force_list"].any():
            plt.plot(self.data["mea_time_list"], self.data["mea_pend_vel_list"], label='mea')
        plt.ylabel('Pendulum Velocity [rad/s]')
        plt.xlabel('Time [sec]')
        plt.legend()
        plt.subplot(5, 1, 5)
        if self.data["des_force_list"].any():
            plt.plot(self.data["des_time_list"], self.data["des_force_list"], label='des')
        if self.data["mea_force_list"].any():
            plt.plot(self.data["mea_time_list"], self.data["mea_force_list"], label='mea')
        plt.ylabel('u | Cart Force [N]')
        plt.xlabel('Time [sec]')
        plt.legend()
        plt.show()

    def polar_plot(self):
        fig, ax = plt.subplots(figsize=(15, 10), subplot_kw={'projection': 'polar'})
        if self.data["des_force_list"].any():
            ax.plot(self.data["des_pend_pos_list"], self.data["des_time_list"], label='des')
        if self.data["mea_force_list"].any():
            ax.plot(self.data["mea_pend_pos_list"], self.data["mea_time_list"], label='mea')
        ax.set_rmax(np.round(self.data["mea_time_list"][-1]))
        # ax.set_rticks([5, 10, 15, 20])  # Less radial ticks
        # ax.set_rgrids([5,10], angle=22)
        ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
        ax.set_theta_offset(np.pi / 2.0)
        ax.grid(True)
        ax.set_title("Pendulum Position", va='bottom')
        plt.legend()
        plt.show()
