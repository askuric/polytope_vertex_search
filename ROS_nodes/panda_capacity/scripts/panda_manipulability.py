#!/usr/bin/env python
import rospy
import tf
import numpy as np
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
# robot module import
from capacity.robot_solver import RobotSolver 
# capacity visualisation utils
import capacity.capacity_visual_utils as capacity_utils

# initial joint positions
joint_positions = [0,0,0,0,0,0,0]

# instance of robot solver
panda = RobotSolver("panda_link0", "panda_link8")
# getting the node namespace
namespace = rospy.get_namespace()
# base frame name
frame = namespace+panda.base_link


# function receiveing the new joint positions
def callback(data):
    global joint_positions
    joint_positions = data.position

# function drawing the marker upon receiving new joint positions
def draw_ellipsoids(q, scaling_vel = 5, scaling_force = 500):

    # calculate dk of the robot
    # just for visualisation - to display the ellipsoid on the tip of the robot
    pose = panda.dk_position(q)
    
    # calculate manip velocity
    sing_val, U = panda.manipulability_velocity(q)
    publish_manip_vel = rospy.Publisher(namespace+'panda/manip_velocity', Marker, queue_size=10)
    publish_manip_vel.publish(capacity_utils.create_ellipsoid_msg(sing_val, U, pose, frame, scaling_vel))

    # calculate manip force
    sing_val, U = panda.manipulability_force(q)
    publish_manip_force = rospy.Publisher(namespace+'panda/manip_force', Marker, queue_size=10)
    publish_manip_force.publish(capacity_utils.create_ellipsoid_msg(sing_val, U, pose, frame, scaling_force))

# main funciton of the class
def panda_manipulability():
    global joint_positions
    rospy.init_node('panda_manipulability', anonymous=True)
    rospy.Subscriber(namespace+"joint_states", JointState, callback)


    rate = rospy.Rate(35) # 10hz
    while not rospy.is_shutdown():
        draw_ellipsoids(joint_positions)
        rate.sleep()

# class definition
if __name__ == '__main__':
    try:
        panda_manipulability()
    except rospy.ROSInterruptException:
        pass
