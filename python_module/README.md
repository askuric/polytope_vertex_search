
## Python capacity module

In this directory you can find the generic robot capacity calculation module called `robot_capacity_solver` which you can easily integrate in your python project, for example
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
def force_polytope_sum_ordered(Jacobian1, Jacobian2, t1_max, t1_min, t2_max, t2_min):
```
See `demo_notebook.ipynb` for one example use case of the module.
