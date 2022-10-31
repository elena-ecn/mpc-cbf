"""Configurations for the MPC controller."""

import numpy as np

sim_time = 100                             # Total simulation time steps
Ts = 0.1                                   # Sampling time [s]
T_horizon = 20                             # Prediction horizon time steps

gamma = 0.5                                # CBF parameter in [0,1]
x0 = np.array([0, 0, 0])                   # Initial state

# Actuator limits
v_limit = 0.26                             # Linear velocity limit
omega_limit = 1.8                          # Angular velocity limit

# Type of control
controller = "MPC-DC"                     # Options: "MPC-CBF", "MPC-DC"
control_type = "setpoint"             # Options: "setpoint", "traj_tracking"
trajectory = "infinity"                    # Type of trajectory. Options: circular, infinity

# For setpoint control:
goal = [2, 1, np.pi/2]                     # Robot's goal for set point control
Q_sp = np.diag([15, 15, 0.005])            # State cost matrix
R_sp = np.array([2, 0.5])                  # Controls cost matrix

# For trajectory tracking control:
Q_tr = np.diag([150, 150, 0.005])          # State cost matrix
R_tr = np.array([2, 0.5])                  # Controls cost matrix

# Obstacles
obstacles_on = False                        # Whether to have obstacles or not
moving_obstacles_on = True                 # Whether to have moving obstacles or not
r = 0.1                                    # Robot radius (for obstacle avoidance)

r_moving_obs = 0.1
# obs = [(-0.2, 0.8, 0.03),
#        (0.0, -0.75, 0.02)]               # Define obstacles as list of tuples (x,y,radius)

# Go-to-goal scenarios: 1-3, 6
# Trajectory tracking scenarios: 4-5
scenario = 4                               # Options: 1-6 or None


# ------------------------------------------------------------------------------
if scenario == 1:
    # Scenario 1
    control_type = "setpoint"
    obs = [(1.0, 0.5, 0.05)]               # Define obstacles as list of tuples (x,y,radius)
elif scenario == 2:
    # Scenario 2
    control_type = "setpoint"
    obs = [(0.5, 0.3, 0.05),
           (1.5, 0.7, 0.05)]               # Define obstacles as list of tuples (x,y,radius)
elif scenario == 3:
    # Scenario 3
    control_type = "setpoint"
    obs = [(0.25, 0.1, 0.025),
           (0.75, 0.2, 0.05),
           (0.6, 0.6, 0.05),
           (1.7, 0.9, 0.05),
           (1.2, 0.6, 0.04)]               # Define obstacles as list of tuples (x,y,radius)
elif scenario == 4:
    # Scenario 4
    control_type = "traj_tracking"
    trajectory = "circular"
    Q_tr = np.diag([1000, 1000, 0.005])    # State cost matrix
    R_tr = np.array([0.1, 0.01])           # Controls cost matrix
    obs = [(-0.2, 0.8, 0.04),
           (0.1, -0.8, 0.03)]              # Define obstacles as list of tuples (x,y,radius)
elif scenario == 5:
    # Scenario 5
    control_type = "traj_tracking"
    trajectory = "infinity"
    # DC
    Q_tr = np.diag([2000, 2000, 0.002])    # State cost matrix
    R_tr = np.array([0.1, 0.00001])        # Controls cost matrix
    # CBF
    Q_tr = np.diag([2500, 2500, 0.005])    # State cost matrix
    R_tr = np.array([0.1, 0.0001])         # Controls cost matrix
    obs = [(-0.5, -0.3, 0.001)]            # Define obstacles as list of tuples (x,y,radius)

# ------------------------------------------------------------------------------
if control_type == "setpoint":
    Q = Q_sp
    R = R_sp
elif control_type == "traj_tracking":
    Q = Q_tr
    R = R_tr
    if trajectory == "circular":
        A = 0.8                            # Amplitude
        w = 0.3                            # Angular frequency
    else:
        A = 1.0                            # Amplitude
        w = 0.3                            # Angular frequency
        x0 = np.array([1, 0, np.pi/2])     # Initial state
else:
    raise ValueError("Please choose among the available options for the control type!")
