#!/usr/bin/env python
import rospy
# time evaluation
import time
# polygone messages
from sensor_msgs.msg import JointState, PointCloud 
from jsk_recognition_msgs.msg import PolygonArray

# robot module import
from capacity.robot_solver import RobotSolver 
# capacity visualisation utils
import capacity.capacity_visual_utils as capacity_utils

# initial joint states
joint_positions1 = [0,0,0,0,0,0,0]
joint_positions2 = [0,0,0,0,0,0,0]

# instance of robot solver
panda = RobotSolver("panda_link0", "panda_link8")
namespace = "/panda_table/"
frame = namespace+panda.base_link

# function drawing the marker upon receiving new joint positions
def callback1(data):
    global joint_positions1
    joint_positions1 = data.position

def callback2(data):
    global joint_positions2
    joint_positions2 = data.position

def plot_polytope(q1,q2, scaling_factor = 500):

    # calculate dk of the robot
    # just for visualisation - to display the ellipsoid on the tip of the robot
    pose = panda.dk_position(q1)
    # calculate force vertexes
    start = time.time()
    force_vertex, force_polytopes = panda.force_polytope_sum(q1,q2)
    print time.time() - start

    # publish plytope
    publish_force_polytope = rospy.Publisher('/combined/panda/force_polytope', PolygonArray, queue_size=1)
    publish_force_polytope.publish(capacity_utils.create_polytopes_msg(force_polytopes, pose, frame, scaling_factor))

    # publish plytop
    publish_force_polytop_vertex = rospy.Publisher('/combined/panda/force_polytope_vertex', PointCloud, queue_size=1)
    publish_force_polytop_vertex.publish(capacity_utils.create_vertex_msg(force_vertex, pose, frame, scaling_factor))
    



# main funciton of the class
def panda_polytope_sum():
    global joint_positions1
    global joint_positions2
    rospy.init_node('panda_polytope_sum')
    rospy.Subscriber("/panda_table/joint_states", JointState, callback1, queue_size= 2)
    rospy.Subscriber("/panda_box/joint_states", JointState, callback2, queue_size= 2)

    rate = rospy.Rate(25) # 100hz
    while not rospy.is_shutdown():
        plot_polytope(joint_positions1,joint_positions2)
        rate.sleep()

# class definition
if __name__ == '__main__':
    try:
        panda_polytope_sum()
    except rospy.ROSInterruptException:
        pass