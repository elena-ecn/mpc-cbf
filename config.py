"""Configurations for the MPC controller."""

import numpy as np

sim_time = 200                             # Total simulation time steps
Ts = 0.1                                   # Sampling time [s]
T_horizon = 20                             # Prediction horizon time steps

gamma = 0.1                                # CBF parameter in [0,1]
safety_dist = 0.03                         # Safety distance
x0 = np.array([0, 0, 0])                   # Initial state

# Actuator limits
v_limit = 0.26                             # Linear velocity limit
omega_limit = 1.8                          # Angular velocity limit

# Type of control
controller = "MPC-CBF"                     # Options: "MPC-CBF", "MPC-DC"
control_type = "setpoint"                  # Options: "setpoint", "traj_tracking"
trajectory = "infinity"                    # Type of trajectory. Options: circular, infinity

# For setpoint control:
goal = [2, 1, np.pi/2]                     # Robot's goal for set point control
Q_sp = np.diag([15, 15, 0.005])            # State cost matrix
R_sp = np.array([2, 0.5])                  # Controls cost matrix

# For trajectory tracking control:
Q_tr = np.diag([200, 200, 0.005])          # State cost matrix
R_tr = np.array([0.1, 0.001])              # Controls cost matrix

# Obstacles
static_obstacles_on = True                 # Whether to have obstacles or not
moving_obstacles_on = False                # Whether to have moving obstacles or not
r = 0.1                                    # Robot radius (for obstacle avoidance)

# Define moving obstacles as list of tuples (ax,bx,ay,by,radius)
# where each obstacle follows a linear trajectory x=ax*t+bx, y=ay*t+by
moving_obs = [(0.2, 0, 0, 0.6, 0.1),
              (-0.15, 1.1, 0, 0.4, 0.05),
              (0.2, -0.5, 0, 0.8, 0.1),
              (0, 1.0, 0.14, -0.8, 0.1),
              (0, 1.8, -0.09, 1.0, 0.1)]

# Static obstacles
# obs = [(-0.2, 0.8, 0.03),
#        (0.0, -0.75, 0.02)]               # Define obstacles as list of tuples (x,y,radius)


scenario = 1                               # Options: 1-6 or None


# ------------------------------------------------------------------------------
if scenario == 1:
    control_type = "setpoint"
    obs = [(1.0, 0.5, 0.1)]               # Define obstacles as list of tuples (x,y,radius)
elif scenario == 2:
    control_type = "setpoint"
    obs = [(0.5, 0.3, 0.1),
           (1.5, 0.7, 0.1)]               # Define obstacles as list of tuples (x,y,radius)
elif scenario == 3:
    control_type = "setpoint"
    obs = [(0.25, 0.2, 0.025),
           (0.75, 0.15, 0.1),
           (0.6, 0.6, 0.1),
           (1.7, 0.9, 0.15),
           (1.2, 0.6, 0.08)]               # Define obstacles as list of tuples (x,y,radius)
elif scenario == 4:
    control_type = "traj_tracking"
    trajectory = "circular"
    gamma = 0.1
    R_tr = np.array([0.1, 0.01])        # Controls cost matrix
    Q_tr = np.diag([800, 800, 2])    # State cost matrix
    obs = [(-0.2, 0.8, 0.1),
           (0.1, -0.8, 0.1)]               # Define obstacles as list of tuples (x,y,radius)
elif scenario == 5:
    control_type = "traj_tracking"
    trajectory = "infinity"
    static_obstacles_on = False
elif scenario == 6:
    control_type = "setpoint"
    static_obstacles_on = False
    moving_obstacles_on = True
    sim_time = 300
    gamma = 0.06

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
    elif trajectory == "infinity":
        A = 1.0                            # Amplitude
        w = 0.3                            # Angular frequency
        x0 = np.array([1, 0, np.pi/2])     # Initial state
else:
    raise ValueError("Please choose among the available options for the control type!")
