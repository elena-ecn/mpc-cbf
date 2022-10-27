"""MPC-CBF controller for a differential drive mobile robot.

Author: elena-ecn
Date: Fall 2022
"""

from mpc_cbf import MPC
from plotter import Plotter

import numpy as np


def main():
    np.random.seed(99)

    # Define controller & simulator
    controller = MPC()
    controller.run_simulation()  # Closed-loop control simulation

    # Plots
    plotter = Plotter(controller.mpc)
    plotter.plot_results()
    plotter.plot_predictions()
    plotter.plot_path()
    plotter.create_animation()


if __name__ == '__main__':
    main()
