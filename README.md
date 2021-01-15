# On-line force capability evaluation based on efficient polytope vertex search

*by Antun Skuric, Vincent Padois, David Daney*<br>
*Submitted to ICRA2021*

<img src="images/rviz_screenshot_2020.png" height="250px">

New on-line polytope vertex search algorithm optimised for force and velocity polytope evaluation of serial robots.

This repository consists of 
- [Matlab/octave implementation](#matlab-octave-testing-scripts)
    - full modular algorithm implementation
    - benchmarking comparison of three vertex search algorithms for *FRANKA Emika Panda* and *UR5* robot
- [Python module for calculating robot capacity](#python-capacity-module)
    - force polytope
    - force/velocity manipulability ellipsoid
    - quick demo jupyter script
- [ROS node for *FRANKA Emika Panda* robot](#ros-panda-capacity-package)



## Matlab / Octave testing scripts

In the `matlab_octave` directory you can find two testing scripts intended for benchmarking of vertex search algorithms
```matlab
panda_test_polytope_algorithms
``` 
and 
```
ur5_test_polytope_algorithms
```

They are both testing the performance of three vertex search algorithms on *Franka Emika Panda* robot and *UR5* robot:

> *On-line force capability evaluation based on eﬀicient polytope vertex search <br> by Antun Skuric, Vincent Padois, David Daney*
```matlab
[f_vert, matrix_inverse_count] = polytope_auctus(Jacobian_mat,tau_min,tau_max);
```

> *Evaluation of Force Capabilities for Redundant manipulatiors <br> by
P.Chiacchio, Pierrot et al.*
```matlab
[f_vert, matrix_inverse_count] = polytope_pierrot(Jacobian_mat,tau_min,tau_max);
```

> *Vertex search algorithm of convex polyhedron representing upperlimb manipulation ability  <br> by Sasaki et al.*
```matlab
[f_vert, matrix_inverse_count] = polytope_sasaki(Jacobian_mat,tau_min,tau_max);
```

## Python capacity module

In the directory `python_module` you can find the generic robot capacity calculation module called `robot_capacity_solver` which you can easily integrate in your python project, for example
```python
import robot_capacity_solver as capacity
```
This module has set of functions for calculation and visualization of robot force capabilities
```python
# velocity manipulability calculation
# returns list with singular values and matrix U
def manipulability_velocity(Jacobian, dq_max):
# force manipulability calculation
# returns list with singular values and matrix U
def manipulability_force(Jacobian, t_max):
```
```python
# force polytope vertex search
# returns list of force vertices and torque vertices
def force_polytope_auctus(Jacobian, t_max, t_min, gravity = None):
#  structured polytope function for visualisaiton
# returns the force vertices and face polygons with ordered vertices
def force_polytope_ordered(Jacobian, t_max, t_min, gravity = None):
```
```python
# interseciton of force polytopes of two robots
# returns list of force vertices and torque vertices
def force_polytope_intersection_auctus(Jacobian1, Jacobian2, t1_max, t1_min, t2_max, t2_min):
# structured interseciton of force polytopes of two robots 
# returns the force vertices and face polygons with ordered vertices
def force_polytope_intersection_ordered(Jacobian1, Jacobian2, t1_max, t1_min, t2_max, t2_min):
```
```python
# structured minkowski sum of force polytopes of two robots 
# returns the force vertices and face polygons with ordered vertices
def force_polytope_sum_auctus(Jacobian1, Jacobian2, t1_max, t1_min, t2_max, t2_min):
```
See [`demo_notebook.ipynb`](python_module/demo_notebook.ipynb) for one example use case of the module.

## ROS panda capacity package
