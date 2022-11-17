from do_mpc.data import save_results, load_results

import config
from mpc_cbf import MPC
from plotter import plot_path_comparisons, plot_cost_comparisons, plot_min_distance_comparison


class MovingObs:
    def __init__(self):
        self.height = 1
        self.width = 1.5
        self.step = 0.02
        self.height_corner = 0.1*self.height
        self.x_tr = 0
        self.y_tr = self.height_corner + self.step
        self.counter = 0
        self.x_mov_obs = 0
        self.y_mov_obs = self.height/2

    def get_square_trajectory_obs(self, t_now):

        t = t_now/config.Ts + 1
        if self.counter < 2:
            self.counter += 1
            return 0, 0

        n_steps_vertical = (self.height - 2*self.height_corner)/self.step
        n_steps_corner = self.height_corner/self.step
        n_steps_horizontal = (self.width - 2*self.height_corner)/self.step
        step = 0.4*self.step

        t = int(t) % (2*n_steps_vertical + 4*n_steps_corner + 2*n_steps_horizontal - 3)

        if self.get_path_segment(self.x_mov_obs, self.y_mov_obs, step) == 'vertical1':
            self.x_mov_obs = 0
            self.y_mov_obs += step

        elif self.get_path_segment(self.x_mov_obs, self.y_mov_obs, step) == 'corner1':
            self.x_mov_obs += step
            self.y_mov_obs += step

        elif self.get_path_segment(self.x_mov_obs, self.y_mov_obs, step) == 'horizontal1':
            self.x_mov_obs += step
            self.y_mov_obs = self.height

        elif self.get_path_segment(self.x_mov_obs, self.y_mov_obs, step) == 'corner2':
            self.x_mov_obs += step
            self.y_mov_obs -= step

        elif self.get_path_segment(self.x_mov_obs, self.y_mov_obs, step) == 'vertical2':
            self.x_mov_obs = self.width
            self.y_mov_obs -= step

        elif self.get_path_segment(self.x_mov_obs, self.y_mov_obs, step) == 'corner3':
            self.x_mov_obs -= step
            self.y_mov_obs -= step

        self.x_mov_obs = round(self.x_mov_obs, 4)
        self.y_mov_obs = round(self.y_mov_obs, 4)

        return self.x_mov_obs, self.y_mov_obs

    def get_path_segment(self, x, y, step):
        if x == 0 and self.height_corner <= y < self.height - self.height_corner:
            path_segment = 'vertical1'
        elif 0 <= x < self.height_corner and self.height - self.height_corner <= y < self.height:
            path_segment = 'corner1'
        elif self.height_corner <= x < self.width - self.height_corner and y <= self.height + step/2:
            path_segment = 'horizontal1'
        elif self.width - self.height_corner <= x < self.width and self.height - self.height_corner < y <= self.height:
            path_segment = 'corner2'
        elif x <= self.width + step and self.height_corner < y <= self.height - self.height_corner:
            path_segment = 'vertical2'
        else:
            path_segment = 'not_defined'

        return path_segment

    def get_square_trajectory(self, t_now):

        t = t_now/config.Ts + 1
        if self.counter < 2:
            self.counter += 1
            return self.x_tr, self.y_tr

        n_steps_vertical = (self.height - 2*self.height_corner)/self.step
        n_steps_corner = self.height_corner/self.step
        n_steps_horizontal = (self.width - 2*self.height_corner)/self.step

        t = int(t) % (2*n_steps_vertical + 4*n_steps_corner + 2*n_steps_horizontal - 3)

        if t < n_steps_vertical - 1:
            self.x_tr = 0
            self.y_tr += self.step
        elif t < n_steps_vertical + n_steps_corner - 1:
            self.x_tr += self.step
            self.y_tr += self.step

        elif t < n_steps_vertical + n_steps_corner + n_steps_horizontal - 3:
            self.x_tr += self.step
            self.y_tr = self.height

        elif t < n_steps_vertical + 2*n_steps_corner + n_steps_horizontal - 2:
            self.x_tr += self.step
            self.y_tr -= self.step

        elif t < 2*n_steps_vertical + 2*n_steps_corner + n_steps_horizontal - 3:
            self.x_tr = self.width
            self.y_tr -= self.step
        elif t < 2*n_steps_vertical + 3*n_steps_corner + n_steps_horizontal - 3:
            self.x_tr -= self.step
            self.y_tr -= self.step

        elif t < 2*n_steps_vertical + 3*n_steps_corner + 2*n_steps_horizontal - 4:
            self.x_tr -= self.step
            self.y_tr = 0
        elif t < 2*n_steps_vertical + 4*n_steps_corner + 2*n_steps_horizontal - 3:
            self.x_tr -= self.step
            self.y_tr += self.step

        self.x_tr = round(self.x_tr, 2)
        self.y_tr = round(self.y_tr, 2)

        return self.x_tr, self.y_tr


def save_mpc_results(controller):
    """Save results in pickle file."""
    if config.controller == "MPC-CBF":
        filename = config.controller + '_' + config.control_type + '_gamma' + str(config.gamma)
    else:
        filename = config.controller + '_' + config.control_type

    save_results([controller.mpc, controller.simulator], result_name=filename)


def load_mpc_results(filename):
    """Load results from pickle file."""
    return load_results('./results/' + filename + '.pkl')


def compare_controller_results(N, gamma):
    """Compares the total cost and min distances for each method over N experiments."""

    obs = [(1.0, 0.5, 0.1)]  # The obstacles used when creating the experiments

    # Get costs & min distances from results
    costs_cbf = []
    costs_dc = []
    min_distances_cbf = []
    min_distances_dc = []
    for i in range(1, N+1):
        # Filename prefix
        if len(str(i)) == 1:
            num = '00' + str(i)
        elif len(str(i)) == 2:
            num = '0' + str(i)
        else:
            num = str(i)

        # Get cbf result
        filename_cbf = num + "_MPC-CBF_setpoint_gamma" + str(gamma)
        results_cbf = load_mpc_results(filename_cbf)
        total_cost_cbf = sum(results_cbf['mpc']['_aux'][:, 1])
        costs_cbf.append(total_cost_cbf)
        positions = results_cbf['mpc']['_x']
        distances = []
        for p in positions:
            distances.append(((p[0]-obs[0][0])**2 + (p[1]-obs[0][1])**2)**(1/2) - (config.r + obs[0][2]))
        min_distances_cbf.append(min(distances))

        # Get dc result
        filename_dc = num + "_MPC-DC_setpoint"
        results_dc = load_mpc_results(filename_dc)
        total_cost_dc = sum(results_dc['mpc']['_aux'][:, 1])
        costs_dc.append(total_cost_dc)
        positions = results_dc['mpc']['_x']
        distances = []
        for p in positions:
            distances.append(((p[0]-obs[0][0])**2 + (p[1]-obs[0][1])**2)**(1/2) - (config.r + obs[0][2]))
        min_distances_dc.append(min(distances))

    # Plot cost comparisons
    plot_cost_comparisons(costs_dc, costs_cbf, gamma)

    # Plot min distances comparison
    plot_min_distance_comparison(min_distances_cbf, min_distances_dc, gamma)

    print("Average cost over all experiments: cbf={}, dc={}".format(sum(costs_cbf)/len(costs_cbf), sum(costs_dc)/len(costs_dc)))
    print("Average min distance over all experiments: cbf={}, dc={}".format(sum(min_distances_cbf)/len(min_distances_cbf),
                                                                            sum(min_distances_dc)/len(min_distances_dc)))


def run_multiple_experiments(N):
    """Runs N experiments for each method."""

    # Run experiments
    cont_type = ["MPC-CBF", "MPC-DC"]
    for c in cont_type:
        config.controller = c
        for i in range(N):
            # Define controller & run simulation
            controller = MPC()
            controller.run_simulation()  # Closed-loop control simulation

            # Store results
            save_mpc_results(controller)


def compare_results_by_gamma():
    """Plots path for each method and different gamma values."""

    results = [load_mpc_results("MPC-DC_setpoint")]
    for gamma in [0.1, 0.2, 0.3, 1.0]:
        filename_cbf = "MPC-CBF_setpoint_gamma" + str(gamma)
        results.append(load_mpc_results(filename_cbf))

    plot_path_comparisons(results)
