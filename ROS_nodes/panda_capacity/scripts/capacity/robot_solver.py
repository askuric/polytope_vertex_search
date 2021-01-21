#!/usr/bin/env python
import numpy as np

# polytope python module
import capacity_solver

# minkowski sum
from scipy.spatial import ConvexHull, Delaunay

# URDF parsing an kinematics 
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import hrl_geom.transformations as trans


class RobotSolver:

    base_link = ""
    tip_link = ""
    robot_urdf = ""
    kdl_kin = ""

    # constructor - reading urdf and constructing the robots kinematic chain
    def __init__(self, base, tip):
        self.base_link = base
        self.tip_link = tip
        
        # loading the root urdf from robot_description parameter
        self.robot_urdf = URDF.from_parameter_server() 
        self.kdl_kin = KDLKinematics(self.robot_urdf , self.base_link, self.tip_link)


        # https://frankaemika.github.io/docs/control_parameters.html
        # maximal torques
        self.t_max =  np.array([self.kdl_kin.joint_limits_effort]).T 
        self.t_min = -self.t_max
        # maximal joint velocities
        self.dq_max = np.array([self.kdl_kin.joint_limits_velocity]).T
        self.dq_min = -self.dq_max
        # maximal joint angles
        self.q_max = np.array([self.kdl_kin.joint_limits_upper]).T
        self.q_min = np.array([self.kdl_kin.joint_limits_lower]).T



    # direct kinematics functions for 7dof
    def forward(self, q):
        return self.kdl_kin.forward(q)

    def dk_position(self, q):
        return self.forward(q)[:3,3]

    def dk_orientation_matrix(self, q):
        return self.forward(q)[0:3,0:3]

    def jacobian(self, q):
        Jac = self.kdl_kin.jacobian(q)
        return Jac

    def jacobian_position(self, q):
        return self.jacobian(q)[:3, :]

    def jacobian_pseudo_inv(self, q):
        return np.linalg.pinv(self.jacobian(q))

    # iterative solving of inverse kinematics
    def ik_solve(self, x_d, q_0, iterations):
        return  self.kdl_kin.inverse(x_d, q_0)

    def gravity_torque(self, q):
        return np.array([self.kdl_kin.gravity(q)]).T



    # velocity manipulability calculation
    def manipulability_velocity(self, q, direction = None):
        # avoid 0 angles
        joint_pos = np.array(q)
        joint_pos[joint_pos==0] = 10**-7

        # jacobian calculation
        if direction is None:
            Jac = self.jacobian_position(q)
        else:
            Jac = np.array(direction).dot(self.jacobian_position(q))
        # use the capacity module
        return capacity_solver.manipulability_velocity(Jac,self.dq_max)

    # force manipulability calculation
    def manipulability_force(self, q, direction = None):
        # avoid 0 angles
        joint_pos = np.array(q)
        joint_pos[joint_pos==0] = 10**-7

        # jacobian calculation
        if direction is None:
            Jac = self.jacobian_position(q)
        else:
            Jac = np.array(direction).dot(self.jacobian_position(q))
        # use the capacity module
        return capacity_solver.manipulability_force(Jac, self.t_max)

    # maximal end effector force
    def force_polytope_intersection(self, q1,q2):
        # jacobian calculation
        Jac1 = self.jacobian_position(q1)
        Jac2 = self.jacobian_position(q2)
        return capacity_solver.force_polytope_intersection_ordered(Jac1,Jac2,self.t_max,self.t_min,self.t_max,self.t_min, self.gravity_torque(q1), self.gravity_torque(q2))


    # maximal end effector force
    def force_polytope_sum(self, q1,q2):
        # jacobian calculation
        Jac1 = self.jacobian_position(q1)
        Jac2 = self.jacobian_position(q2)
        return capacity_solver.force_polytope_sum_ordered(Jac1,Jac2,self.t_max,self.t_min,self.t_max,self.t_min, self.gravity_torque(q1), self.gravity_torque(q2))



    # maximal end effector force
    def force_polytope(self, q, direction = None, force = None):

        # jacobian calculation
        Jac_full = self.jacobian_position(q)
        if direction is not None:
            Jac = np.array(direction).dot(Jac_full)
        else:
            Jac = Jac_full
        
        return capacity_solver.force_polytope_ordered(Jac, self.t_max, self.t_min, self.gravity_torque(q))
