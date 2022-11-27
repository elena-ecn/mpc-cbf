from do_mpc.data import save_results, load_results

import config
from mpc_cbf import MPC
from plotter import plot_path_comparisons, plot_cost_comparisons, plot_min_distance_comparison


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
