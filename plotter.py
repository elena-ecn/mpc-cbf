import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, ImageMagickWriter
from matplotlib.patches import Circle
import do_mpc
import seaborn as sns
import numpy as np
import pandas as pd

import config


class Plotter:
    def __init__(self, controller):
        self.controller = controller
        self.mpc = controller.mpc

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
            ax[0].plot(self.mpc.data['_time'], self.mpc.data['_tvp', 'x_set_point'], 'k--', lw=1)
            ax[0].plot(self.mpc.data['_time'], self.mpc.data['_tvp', 'y_set_point'], 'k--', lw=1)

        # Plot actuator limits
        colors = sns.color_palette()
        ax[1].hlines(y=config.v_limit, xmin=0, xmax=len(self.mpc.data['_time'])*config.Ts, linewidth=1, color=colors[0], linestyle='--', label='v limit')
        ax[1].hlines(y=config.omega_limit, xmin=0, xmax=len(self.mpc.data['_time'])*config.Ts, linewidth=1, color=colors[1], linestyle='--', label='$\omega$ limit')
        ax[1].hlines(y=-config.v_limit, xmin=0, xmax=len(self.mpc.data['_time'])*config.Ts, linewidth=1, color=colors[0], linestyle='--')
        ax[1].hlines(y=-config.omega_limit, xmin=0, xmax=len(self.mpc.data['_time'])*config.Ts, linewidth=1, color=colors[1], linestyle='--')

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

    def create_trajectories_animation(self):
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
        anim.save('images/trajectories_animation.gif', writer=ImageMagickWriter(fps=3))

    def update(self, t_ind, mpc_graphics):
        """Plots the results and predictions at time t_ind for the animation of the predictions."""
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

        # Plot robot in final position
        ax.add_patch(plt.Circle((self.mpc.data['_x'][-1, 0], self.mpc.data['_x'][-1, 1]), config.r, color='b', zorder=2))

        # Plot goal or reference trajectory
        if config.control_type == "setpoint":
            ax.plot(config.goal[0], config.goal[1], 'g*', label="Goal")
        else:
            ax.plot(self.mpc.data['_tvp', 'x_set_point'], self.mpc.data['_tvp', 'y_set_point'], 'k--', label="Reference trajectory", zorder=0)

        # Plot moving obstacle trajectory
        if config.moving_obstacles_on is True:
            for i in range(len(config.moving_obs)):
                # Plot final position
                ax.add_patch(plt.Circle((self.mpc.data['_tvp', 'x_moving_obs'+str(i)][-1],
                                         self.mpc.data['_tvp', 'y_moving_obs'+str(i)][-1]),
                                        config.moving_obs[i][4], color='k'))
                # Plot path
                ax.plot(self.mpc.data['_tvp', 'x_moving_obs'+str(i)], self.mpc.data['_tvp', 'y_moving_obs'+str(i)],
                        'k:', label="Moving Obstacle path", alpha=0.3)

        # Plot static obstacles
        if config.static_obstacles_on:
            for x_obs, y_obs, r_obs in config.obs:
                ax.add_patch(plt.Circle((x_obs, y_obs), r_obs, color='k'))

        # Only show unique legends
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        plt.legend(by_label.values(), by_label.keys(), loc="upper left")

        plt.savefig('images/path.png')
        plt.show()

    def plot_cbf(self):
        """Plots the CBF values."""

        if self.controller.static_obstacles_on or self.controller.moving_obstacles_on:
            cbfs = []
            if self.controller.static_obstacles_on:
                for i in range(len(self.controller.obs)):
                    h = []
                    for x in self.mpc.data['_x']:
                        h.append(self.controller.h(x, self.controller.obs[i]))
                    cbfs.append(h)

            cbfs_mov = []
            if self.controller.moving_obstacles_on:
                for i in range(len(self.controller.moving_obs)):
                    h = []
                    for x in self.mpc.data['_x']:
                        obs = (self.mpc.data['_tvp', 'x_moving_obs'+str(i)][i], self.mpc.data['_tvp', 'y_moving_obs'+str(i)][i], self.controller.moving_obs[i][4])
                        h.append(self.controller.h(x, obs))
                    cbfs_mov.append(h)

            sns.set_theme()
            fig, ax = plt.subplots(figsize=(9, 5))
            for i in range(len(cbfs)):
                ax.plot(cbfs[i], label="h_obs"+str(i))
            for i in range(len(cbfs_mov)):
                ax.plot(cbfs_mov[i], label="h_mov_obs"+str(i))
            plt.axhline(y=0, color='k', linestyle='--')
            ax.set_xlabel('Time [s]')
            ax.set_ylabel('h [m]')
            plt.title("CBF Values")
            plt.tight_layout()
            plt.legend()
            plt.savefig('images/cbf.png')
            plt.show()

    def create_path_animation(self):
        """Creates an animation for the robot path in the x-y plane."""
        global ax

        sns.set_theme()
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        plt.title("Robot path")
        plt.tight_layout()
        ax.axis('equal')
        offset = 0.5
        ax.set_xlim([min(self.mpc.data['_x'][:, 0])-offset, max(self.mpc.data['_x'][:, 0])+offset])
        ax.set_ylim([min(self.mpc.data['_x'][:, 1])-offset, max(self.mpc.data['_x'][:, 1])+offset])

        # Plot goal or reference trajectory
        if config.control_type == "setpoint":
            ax.plot(config.goal[0], config.goal[1], 'g*', label="Goal")
        else:
            ax.plot(self.mpc.data['_tvp', 'x_set_point'], self.mpc.data['_tvp', 'y_set_point'], 'k--', label="Reference trajectory")

        # Static obstacles
        if config.static_obstacles_on:
            for x_obs, y_obs, r_obs in config.obs:
                ax.add_patch(plt.Circle((x_obs, y_obs), r_obs, color='k'))

        # Moving obstacle
        if config.moving_obstacles_on is True:
            for i in range(len(config.moving_obs)):
                globals()['moving_obs%s' % str(i)] = Circle((0, 0), 0, color='k')
                ax.add_patch(globals()['moving_obs%s' % str(i)])

        # Robot's heading indicator
        globals()['robot_heading'] = plt.Arrow(x=0, y=0, dx=0, dy=0, color='k', width=0.1, linewidth=0.6)
        ax.add_patch(globals()['robot_heading'])

        # Robot base
        globals()['robot_base'] = Circle((0, 0), config.r)
        ax.add_patch(globals()['robot_base'])

        # Robot's trace
        globals()['trace'] = ax.plot([], [], 'b', alpha=0.7, lw=1.5)[0]

        # Legend
        plt.legend(loc="upper left")

        # Run the animation
        ani = FuncAnimation(fig, self.animate_path, frames=len(self.mpc.data['_x'][:, 0]), interval=config.Ts*1000, repeat=False)
        plt.show()
        # Save animation as gif
        ani.save('images/path_animation.gif', writer=ImageMagickWriter(fps=config.sim_time/config.Ts))

    def animate_path(self, i):
        """Draws each frame of the animation."""

        # Robot's heading
        ax.patches.remove(globals()['robot_heading'])
        globals()['robot_heading'] = plt.Arrow(x=self.mpc.data['_x'][i, 0],
                                               y=self.mpc.data['_x'][i, 1],
                                               dx=np.cos(self.mpc.data['_x'][i, 2])/6,
                                               dy=np.sin(self.mpc.data['_x'][i, 2])/6,
                                               color='k',
                                               width=0.1,
                                               linewidth=0.6)
        ax.add_patch(globals()['robot_heading'])

        # Robot's base
        ax.patches.remove(globals()['robot_base'])
        globals()['robot_base'] = Circle((self.mpc.data['_x'][i, 0], self.mpc.data['_x'][i, 1]), config.r, zorder=2)
        ax.add_patch(globals()['robot_base'])

        # Robot's trace
        tx = [t for t in self.mpc.data['_x'][:i, 0]]  # Trace x-axis positions
        ty = [t for t in self.mpc.data['_x'][:i, 1]]  # Trace y-axis positions
        globals()['trace'].set_data(tx, ty)

        # Moving obstacle
        if config.moving_obstacles_on is True:
            for i_obs in range(len(config.moving_obs)):
                ax.patches.remove(globals()['moving_obs%s' % str(i_obs)])
                globals()['moving_obs%s' % str(i_obs)] = Circle((self.mpc.data['_tvp', 'x_moving_obs'+str(i_obs)][i],
                                                                 self.mpc.data['_tvp', 'y_moving_obs'+str(i_obs)][i]),
                                                                config.moving_obs[i_obs][4], color='k', zorder=2)
                ax.add_patch(globals()['moving_obs%s' % str(i_obs)])
        return


def plot_path_comparisons(results, gammas):
    """Plots the robot path for each method and different gamma values."""
    sns.set_theme()
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    plt.title("Robot path (N=10)")
    plt.tight_layout()
    ax.axis('equal')

    # Plot MPC-CBF paths for each gamma
    for i in range(len(results)-1):
        X = results[i+1]['mpc']['_x']
        label = "MPC-CBF ($\gamma={}$)".format(gammas[i])
        ax.plot(X[:, 0], X[:, 1], label=label)

    # Plot MPC-DC path
    X_dc = results[0]['mpc']['_x']
    ax.plot(X_dc[:, 0], X_dc[:, 1], 'k--', label="MPC-DC")

    # Plot initial position
    x0 = X_dc[0, :2]
    ax.plot(x0[0], x0[1], 'b.', label="Initial position")

    # Plot goal
    ax.plot(config.goal[0], config.goal[1], 'g*', label="Goal")

    # Plot static obstacles
    if config.static_obstacles_on:
        for x_obs, y_obs, r_obs in config.obs:
            ax.add_patch(plt.Circle((x_obs, y_obs), r_obs + config.r, color='k'))

    # Only show unique legends
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(), loc="upper left")

    plt.savefig('images/path_comparisons.png')
    plt.show()


def plot_cost_comparisons(costs_dc, costs_cbf, gamma):
    """Plots the objective function cost for each method for all experiments."""

    # Plot cost vs experiment
    sns.set_theme()
    fig, ax = plt.subplots(figsize=(9, 5))
    t = np.linspace(1, len(costs_cbf), num=len(costs_cbf))
    ax.plot(t, costs_cbf, 'o-', label="MPC-CBF ($\gamma={}$)".format(gamma))
    ax.plot(t, costs_dc, 'o-', label="MPC-DC")
    ax.set_xlabel('Experiment')
    ax.set_ylabel('Cost')
    plt.xticks(t)
    plt.title("Total Objective Function Cost (N=10)")
    plt.tight_layout()
    plt.legend(loc="upper right")
    plt.savefig('images/cost_comparisons.png')
    plt.show()

    # Convert data to df
    df_costs_cbf = pd.DataFrame({'Controller': "MPC-CBF ($\gamma={}$)".format(gamma), 'Costs': costs_cbf})
    df_costs_dc = pd.DataFrame({'Controller': "MPC-DC", 'Costs': costs_dc})
    costs_df = pd.concat([df_costs_cbf, df_costs_dc])

    # Plot average cost of all experiments vs method
    sns.set_theme()
    plt.figure(figsize=(9, 5))
    sns.barplot(data=costs_df, x='Controller', y='Costs', capsize=.2)
    plt.title("Total Cost Comparison for {} experiments".format(len(costs_cbf)))
    plt.xlabel('Controller')
    plt.ylabel('Average Total Costs')
    plt.savefig('images/cost_comparisons_barplot.png')
    plt.show()


def plot_min_distance_comparison(min_distances_cbf, min_distances_dc, gamma):
    """Plots the minimum distance for each method for all experiments."""

    # Plot min distance vs experiment
    sns.set_theme()
    fig, ax = plt.subplots(figsize=(9, 5))
    t = np.linspace(1, len(min_distances_cbf), num=len(min_distances_cbf))
    ax.plot(t, min_distances_cbf, 'o-', label="MPC-CBF ($\gamma={}$)".format(gamma))
    ax.plot(t, min_distances_dc, 'o-', label="MPC-DC")
    ax.set_xlabel('Experiment')
    ax.set_ylabel('Minimum distance')
    plt.xticks(t)
    plt.title("Minimum Distances (N=10)")
    plt.tight_layout()
    plt.legend(loc="upper right")
    plt.savefig('images/min_distances_comparisons.png')
    plt.show()

    # Convert data to df
    df_cbf = pd.DataFrame({'Controller': "MPC-CBF ($\gamma={}$)".format(gamma), 'Dist': min_distances_cbf})
    df_dc = pd.DataFrame({'Controller': "MPC-DC", 'Dist': min_distances_dc})
    dist_df = pd.concat([df_cbf, df_dc])

    # Plot average min distance of all experiments vs method
    sns.set_theme()
    plt.figure(figsize=(9, 5))
    sns.barplot(data=dist_df, x='Controller', y='Dist', capsize=.2)
    plt.title("Min distance for {} experiments".format(len(min_distances_cbf)))
    plt.xlabel('Controller')
    plt.ylabel('Average Minimum Distances')
    plt.savefig('images/min_distances_comparisons_barplot.png')
    plt.show()
