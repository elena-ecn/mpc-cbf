# mpc-cbf
Model Predictive Control with discrete-time Control Barrier Functions (MPC-CBF) for a wheeled mobile robot.

The MPC-CBF optimization problem is given by:


$$\\begin{aligned}
\\min\_{u_{t: t+N-1 \\mid t}} \\quad & \\frac{1}{2} \\tilde{x}\_N^T Q_x \\tilde{x}\_N+\\sum\_{k=0}^{N-1} \\frac{1}{2} \\tilde{x}\_k^T Q_x \\tilde{x}\_k+\\frac{1}{2} u\_k^T Q\_u u\_k\\\\
\\textrm{s.t.} \\quad 
 & x\_{t+k+1 \\mid t}=x\_{t+k \\mid t}+f\\left(x\_{t+k \\mid t}, u\_{t+k \\mid t}\\right) \\cdot T\_s, \\quad k=0, . ., N-1,\\\\
 & x\_{\\min } \\leq x\_{t+k \\mid t} \\leq x\_{\\max }, \\quad k=0, \\ldots, N-1,\\\\
 & u\_{\\min } \\leq u\_{t+k \\mid t} \\leq u\_{\\max }, \\quad k=0, \\ldots, N-1,   \\\\
 & x\_{t \\mid t}=x\_t,   \\\\
 & \\Delta h\\left(x\_{t+k \\mid t}, u\_{t+k \\mid t}\\right) \\geq-\\gamma h\\left(x\_{t+k \\mid t}\\right), \\quad k=0, \\ldots, N-1 \\\\
\\end{aligned}$$


where $\\tilde{x}\_k=x\_{des,k} - x\_{k}$.


Results
-------

### Scenario 1
**Path comparison for different values of γ for MPC-CBF and with MPC-DC**
<p align="center" width="100%">
    <img src="images/Display/path_comparisons.png" width="400">
    <br>Path comparison
</p>

**MPC-CBF**
<p align="center" width="100%">
    <img src="images/Display/path_scenario1.png" width="400">
    <br>Robot path
</p>

<p align="center" width="100%">
    <img src="images/Display/trajectories_animation_scenario1.gif" width="400">
    <br>Trajectories
</p>


### Scenario 3
<p align="center" width="100%">
    <img src="images/Display/path_animation_scenario3.gif" width="400">
    <br>Robot path
</p>

<p align="center" width="100%">
    <img src="images/Display/cbf_scenario3.png" width="400">
    <br>CBF values
</p>

### Scenario 4
<p align="center" width="100%">
    <img src="images/Display/path_animation_scenario4.gif" width="400">
    <br>Robot path
</p>

<p align="center" width="100%">
    <img src="images/Display/cbf_scenario4.png" width="400">
    <br>CBF values
</p>

<p align="center" width="100%">
    <img src="images/Display/trajectories_scenario4.png" width="400">
    <br>Trajectories
</p>


### Scenario 5
<p align="center" width="100%">
    <img src="images/Display/path_animation_scenario5.gif" width="400">
    <br>Robot path
</p>


### Scenario 6
<p align="center" width="100%">
    <img src="images/Display/path_animation_scenario6.gif" width="400">
    <br>Robot path
</p>

<p align="center" width="100%">
    <img src="images/Display/cbf_scenario6.png" width="400">
    <br>CBF values
</p>


### Gazebo simulation with turtlebot3

<p align="center" width="100%">
    <img src="images/Display/s1.gif" width="400">
</p>

<p align="center" width="100%">
    <img src="images/Display/s2.gif" width="400">
</p>





Installation
------------

To use this project, install it locally via:
```
git clone https://github.com/elena-ecn/mpc-cbf.git
```

The dependencies can be installed by running:
```
pip install -r requirements.txt
```

The controller configuration can be changed through the config.py.

To execute the code, run:
```
python3 main.py
```

License
-------
The contents of this repository are covered under the [MIT License](LICENSE).


References
----------
* [[1] J. Zeng, B. Zhang, and K. Sreenath, “Safety-Critical Model Predictive Control with Discrete-Time Control
Barrier Function,” in 2021 American Control Conference (ACC), May 2021, pp. 3882–3889.](https://ieeexplore.ieee.org/document/9483029)