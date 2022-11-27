"""MPC-CBF controller for a differential drive mobile robot.

Author: Elena Oikonomou
Date: Fall 2022
"""

import numpy as np

from mpc_cbf import MPC
from plotter import Plotter
import util


def main():
    np.random.seed(99)

    # Define controller & run simulation
    controller = MPC()
    controller.run_simulation()  # Closed-loop control simulation

    # Plots
    plotter = Plotter(controller)
    plotter.plot_results()
    plotter.plot_predictions()
    plotter.plot_path()
    plotter.create_trajectories_animation()
    plotter.create_path_animation()
    plotter.plot_cbf()

    # Store results
    # util.save_mpc_results(controller)


if __name__ == '__main__':
    main()
