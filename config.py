import numpy as np

sim_time = 100                     # Total simulation time steps
Ts = 0.2                          # Sampling time [s]
T_horizon = 10                     # Prediction horizon time steps

gamma = 0.2                        # CBF parameter
# Initial conditions
x0 = np.array([0, 0, 0])           # Initial state

# Actuator limits
v_limit = 0.26                     # Linear velocity limit
omega_limit = 1.8                  # Angular velocity limit

# Objective function cost matrices
R = np.array([2, 0.5])             # Controls cost matrix

# Type of control
controller = "MPC-CBF"             # Options: "MPC-CBF", "MPC-DC"
control_type = "setpoint"     # Options: "setpoint", "traj_tracking"

# For setpoint control:
goal = [2, 1, np.pi/2]             # Robot's goal for set point control
Q_sp = np.diag([15, 15, 0.005])    # State cost matrix

# For trajectory tracking control:
Q_tr = np.diag([45, 45, 0.001])   # State cost matrix
trajectory = "circular"            # Type of trajectory. Options: circular,

# Obstacles
obstacles_on = True                # Whether to have obstacles or not
r = 0.1                            # Robot radius (for obstacle avoidance)
obs = [(-0.2, 0.8, 0.03),
       (0.0, -0.75, 0.02)]           # Define obstacles as list of tuples (x,y,radius)


# Go-to-goal scenarios
# Scenario 1
# obs = [(1.0, 0.5, 0.05)]           # Define obstacles as list of tuples (x,y,radius)

# Scenario 2
obs = [(0.5, 0.3, 0.05),
       (1.5, 0.7, 0.05)]           # Define obstacles as list of tuples (x,y,radius)

# Scenario 3
# obs = [(0.25, 0.1, 0.025),
#        (0.75, 0.2, 0.05),
#        (0.6, 0.6, 0.05),
#        (1.7, 0.9, 0.05),
#        (1.2, 0.6, 0.04)]           # Define obstacles as list of tuples (x,y,radius)


# ------------------------------------------------------------------------------
if control_type == "setpoint":
    Q = Q_sp
elif control_type == "traj_tracking":
    Q = Q_tr
    if trajectory == "circular":
        A = 0.8                    # Amplitude
        w = 0.32                   # Angular frequency
else:
    raise ValueError("Please choose among the available options for the control type!")
