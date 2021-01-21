#!/usr/bin/env python
import rospy
# time evaluation
import time
#some math
import numpy as np
# polygone messages
from sensor_msgs.msg import JointState, PointCloud 
from jsk_recognition_msgs.msg import PolygonArray
from std_msgs.msg import Float64

# robot module import
from capacity.robot_solver import RobotSolver 
# capacity visualisation utils
import capacity.capacity_visual_utils as capacity_utils

# getting the node namespace
namespace = rospy.get_namespace()

# instance of robot solver
panda = RobotSolver("panda_link0", "panda_link8")
frame = namespace + panda.base_link

# initial joint positions
joint_positions = [0,0,0,0,0,0,0]

# function receiveing the new joint positions
def callback(data):
    global joint_positions
    joint_positions = data.position

def plot_polytope(q, scaling_factor = 500):

    # calculate dk of the robot
    # just for visualisation - to display the ellipsoid on the tip of the robot
    pose = panda.dk_position(q)
   
    # calculate force vertexes
    start = time.time()
    force_vertex, force_polytopes = panda.force_polytope(q)
    print time.time() - start

    # publish vertices
    publish_force_polytop_vertex = rospy.Publisher(namespace+'panda/force_polytope_vertex', PointCloud, queue_size=10)
    publish_force_polytop_vertex.publish(capacity_utils.create_vertex_msg(force_vertex, pose, frame, scaling_factor))

    # publish plytope
    publish_force_polytope = rospy.Publisher(namespace+'panda/force_polytope', PolygonArray, queue_size=10)
    publish_force_polytope.publish(capacity_utils.create_polytopes_msg(force_polytopes, pose, frame, scaling_factor))

    # find maximal force in z direction
    f_v,f_p = panda.force_polytope(q, [0,0,1])
    fmax = np.max(f_v)
    # publish the maximal force
    f_max_msg = Float64(fmax/9.81)
    publish_fmax = rospy.Publisher(namespace+'panda/max_force_z', Float64, queue_size=2)
    publish_fmax.publish(f_max_msg)
    

# main function of the class
def panda_polytope():
    global joint_positions
    rospy.init_node('panda_polytope')

    joint_state_topic = rospy.get_param('joint_state_topic', 'joint_states')
    rospy.Subscriber(namespace + joint_state_topic, JointState, callback, queue_size= 2)

    rate = rospy.Rate(25) #Hz
    while not rospy.is_shutdown():
        plot_polytope(joint_positions)
        rate.sleep()

# class definition
if __name__ == '__main__':
    try:
        panda_polytope()
    except rospy.ROSInterruptException:
        pass