import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, ImageMagickWriter
import do_mpc
import seaborn as sns
import numpy as np

import config


class Plotter:
    def __init__(self, mpc):
        self.mpc = mpc

    def plot_results(self):
        """Plots the state trajectories, the controls and objective value at each timestep."""
        sns.set_theme()
        fig, ax, graphics = do_mpc.graphics.default_plot(self.mpc.data, figsize=(9, 5))
        graphics.plot_results()
        graphics.reset_axes()
        lines = graphics.result_lines['_x']
        ax[0].legend(lines, ['x-position', 'y-position', 'theta'])
        lines = graphics.result_lines['_u']
        ax[1].legend(lines, ['v (linear velocity)', '$\omega$ (angular velocity)'], title='controls')
        ax[2].set_xlabel('Time [s]')
        ax[0].set_ylabel('State')
        ax[1].set_ylabel('Input')
        ax[2].set_ylabel('Cost')
        fig.suptitle('Trajectories', y=1.0)

        # Plot reference trajectory, if trajectory tracking
        if config.control_type == "traj_tracking":
            t = np.linspace(0, config.sim_time*config.Ts)
            if config.trajectory == "circular":
                ax[0].plot(t, config.A*np.cos(config.w*t), 'k--', lw=1)
                ax[0].plot(t, config.A*np.sin(config.w*t), 'k--', lw=1)
            else:
                ax[0].plot(t, config.A*np.cos(config.w*t)/(np.sin(config.w*t)**2 + 1), 'k--', lw=1)
                ax[0].plot(t, config.A*np.sin(config.w*t)*np.cos(config.w*t)/(np.sin(config.w*t)**2 + 1), 'k--', lw=1)

        plt.savefig('images/trajectories.png')
        plt.show()

    def plot_predictions(self, t_ind=int(config.sim_time/2)):
        """Plots the predictions at timestep t_ind."""
        mpc_graphics = do_mpc.graphics.Graphics(self.mpc.data)

        sns.set_theme()
        fig, ax = plt.subplots(2, sharex=True, figsize=(9, 5))
        fig.align_ylabels()
        mpc_graphics.add_line(var_type='_x', var_name='x', axis=ax[0])
        mpc_graphics.add_line(var_type='_u', var_name='u', axis=ax[1])
        lines = mpc_graphics.result_lines['_x']
        ax[0].legend(lines, ['x-position', 'y-position', 'theta'])
        lines = mpc_graphics.result_lines['_u']
        ax[1].legend(lines, ['v (linear velocity)', '$\omega$ (angular velocity)'], title='controls')

        # Plot predictions at specific time index t_ind
        mpc_graphics.plot_results(t_ind=t_ind)
        mpc_graphics.plot_predictions(t_ind=t_ind)

        ax[1].set_xlabel('Time [s]')
        ax[0].set_ylabel('State')
        ax[1].set_ylabel('Input')
        fig.suptitle('Predictions at time t={}s'.format(t_ind*config.Ts))
        plt.savefig('images/predictions.png')
        plt.show()

    def create_animation(self):
        """Creates an animation with the predictions."""
        mpc_graphics = do_mpc.graphics.Graphics(self.mpc.data)
        mpc_graphics.reset_axes()
        
        sns.set_theme()
        fig, ax = plt.subplots(2, sharex=True, figsize=(9, 5))
        fig.align_ylabels()
        mpc_graphics.add_line(var_type='_x', var_name='x', axis=ax[0])
        mpc_graphics.add_line(var_type='_u', var_name='u', axis=ax[1])
        lines = mpc_graphics.result_lines['_x']
        ax[0].legend(lines, ['x-position', 'y-position', 'theta'], loc="upper left")
        lines = mpc_graphics.result_lines['_u']
        ax[1].legend(lines, ['v (linear velocity)', '$\omega$ (angular velocity)'], title='controls', loc="upper left")

        ax[1].set_xlabel('Time [s]')
        ax[0].set_ylabel('State')
        ax[1].set_ylabel('Input')
        fig.suptitle('Trajectories & Predictions')

        anim = FuncAnimation(fig, self.update, frames=config.sim_time, repeat=False, fargs=(mpc_graphics,))
        anim.save('images/anim.gif', writer=ImageMagickWriter(fps=3))

    def update(self, t_ind, mpc_graphics):
        """Plots the results and predictions at time t_ind for the animation."""
        mpc_graphics.plot_results(t_ind)
        mpc_graphics.plot_predictions(t_ind)
        mpc_graphics.reset_axes()

    def plot_path(self):
        """Plots the robot path in the x-y plane."""
        sns.set_theme()
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.plot(self.mpc.data['_x'][:, 0], self.mpc.data['_x'][:, 1], label="Robot path")
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        plt.title("Robot path")
        plt.tight_layout()
        ax.axis('equal')

        # Plot initial position
        ax.plot(config.x0[0], config.x0[1], 'r.', label="Initial position")

        # Plot goal or reference trajectory
        if config.control_type == "setpoint":
            ax.plot(self.mpc.data['_x'][-1, 0], self.mpc.data['_x'][-1, 1], 'b.', label="Final position")
            ax.plot(config.goal[0], config.goal[1], 'g*', label="Goal")
        else:
            t = np.linspace(0, config.sim_time*config.Ts)
            if config.trajectory == "circular":
                ax.plot(config.A*np.cos(config.w*t), config.A*np.sin(config.w*t), 'k--', label="Reference trajectory")
            else:
                ax.plot(config.A*np.cos(config.w*t)/(np.sin(config.w*t)**2 + 1),
                        config.A*np.sin(config.w*t)*np.cos(config.w*t)/(np.sin(config.w*t)**2 + 1),
                        'k--', label="Reference trajectory")

        # Plot (extended) obstacles
        if config.obstacles_on:
            for x_obs, y_obs, r_obs in config.obs:
                ax.add_patch(plt.Circle((x_obs, y_obs), r_obs+config.r, color='k'))

        plt.legend(loc="upper left")
        plt.savefig('images/path.png')
        plt.show()
