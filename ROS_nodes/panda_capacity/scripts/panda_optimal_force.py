#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState, PointCloud 
from geometry_msgs.msg import Point32
# robot module import
import panda_solver as panda
# time evaluation
import time
#arguments
import sys

from visualization_msgs.msg import Marker
import tf
from std_msgs.msg import Header,Float64

#some math
import numpy as np

# dynamic configuration changing
import dynamic_reconfigure.client

joint_positions1 = [0,0,0,0,0,0,0]
joint_positions2 = [0,0,0,0,0,0,0]

# function drawing the marker upon receiving new joint positions
def callback1(data):
    global joint_positions1
    joint_positions1 = data.position

def callback2(data):
    global joint_positions2
    joint_positions2 = data.position


def plot_polytope(q1, q2, force):

    scale_factor = 500

    #f_max1 = panda.max_force_in_direction(q1,[0,0,1])
    #f_max2 = panda.max_force_in_direction(q2,[0,0,1])
    f_v1,f_p1 = panda.force_polytope_ordered(q1,[0,0,1])
    f_v2,f_p2 = panda.force_polytope_ordered(q2,[0,0,1])
    f_max1 = np.max(f_v1)
    f_max2 = np.max(f_v2)

    f_max_sum = f_max1 + f_max2

    f1 = force * f_max1/f_max_sum
    if f1 >= f_max1 - 5:
        f1 = f_max1 - 5
    f2 = force - f1
    if f2 >= f_max2 - 5:
        f2 = f_max2 - 5
        f1 = force - f2
        if f1 >= f_max1 - 5:
            f1 = f_max1 - 5
    
    
    pose1 = panda.dk_position(q1)
    pose2 = panda.dk_position(q2)

    
    marker = Marker()
    marker.header.frame_id = "/panda_table/panda_link0"
    marker.pose.position.x =pose1[0]
    marker.pose.position.y =pose1[1]
    marker.pose.position.z =pose1[2]
    quaternion = tf.transformations.quaternion_from_euler(0,0,0)
    #type(pose) = geometry_msgs.msg.Pose
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    marker.type = marker.CUBE
    marker.color.r = 138.0/255.0
    marker.color.g = 226.0/255.0
    marker.color.b = 52.0/255.0
    marker.color.a = 1
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = f_max1/scale_factor
    marker.pose.position.z =pose1[2]  + f_max1/scale_factor/2
    optimal_force = rospy.Publisher('/panda_table/optimal_force', Marker, queue_size=10)
    optimal_force.publish(marker)

    marker = Marker()
    marker.header.frame_id = "/panda_box/panda_link0"
    marker.pose.position.x =pose2[0]
    marker.pose.position.y =pose2[1]
    marker.pose.position.z =pose2[2]
    quaternion = tf.transformations.quaternion_from_euler(0,0,0)
    #type(pose) = geometry_msgs.msg.Pose
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    marker.type = marker.CUBE
    marker.color.r = 114.0/255.0
    marker.color.g = 159.0/255.0
    marker.color.b = 207.0/255.0
    marker.color.a = 1
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = f_max2/scale_factor
    marker.pose.position.z =pose2[2]  + f_max2/scale_factor/2
    optimal_force = rospy.Publisher('/panda_box/optimal_force', Marker, queue_size=10)
    optimal_force.publish(marker)
    
    marker = Marker()
    marker.header.frame_id = "/panda_table/panda_link0"
    marker.pose.position.x =pose1[0]
    marker.pose.position.y =pose1[1]
    marker.pose.position.z =pose1[2] + f1/scale_factor
    quaternion = tf.transformations.quaternion_from_euler(0,0,0)
    #type(pose) = geometry_msgs.msg.Pose
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    marker.type = marker.SPHERE
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 1 
    marker.color.a = 1
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03
    optimal_force = rospy.Publisher('/panda_table/cmd_force', Marker, queue_size=10)
    optimal_force.publish(marker)

    marker = Marker()
    marker.header.frame_id = "/panda_box/panda_link0"
    marker.pose.position.x =pose2[0]
    marker.pose.position.y =pose2[1]
    marker.pose.position.z =pose2[2] + f2/scale_factor
    quaternion = tf.transformations.quaternion_from_euler(0,0,0)
    #type(pose) = geometry_msgs.msg.Pose
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    marker.type = marker.SPHERE
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 1
    marker.color.a = 1
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03
    optimal_force = rospy.Publisher('/panda_box/cmd_force', Marker, queue_size=10)
    optimal_force.publish(marker)


    marker = Marker()
    marker.header.frame_id = "/panda_table/panda_link0"
    marker.pose.position.x =pose1[0]
    marker.pose.position.y =pose1[1]
    marker.pose.position.z =pose1[2] + force/2/scale_factor
    quaternion = tf.transformations.quaternion_from_euler(0,0,0)
    #type(pose) = geometry_msgs.msg.Pose
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    marker.type = marker.SPHERE
    if f_max1 < force/2:
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1
    else:
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03
    optimal_force = rospy.Publisher('/panda_table/half_force', Marker, queue_size=10)
    optimal_force.publish(marker)

    marker = Marker()
    marker.header.frame_id = "/panda_box/panda_link0"
    marker.pose.position.x =pose2[0]
    marker.pose.position.y =pose2[1]
    marker.pose.position.z =pose2[2] + force/2/scale_factor
    quaternion = tf.transformations.quaternion_from_euler(0,0,0)
    #type(pose) = geometry_msgs.msg.Pose
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    marker.type = marker.SPHERE
    if f_max2 < force/2:
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1
    else:
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03
    optimal_force = rospy.Publisher('/panda_box/half_force', Marker, queue_size=10)
    optimal_force.publish(marker)

    return f1, f2

# main funciton of the class
def panda_optimal_force():
    global joint_positions1
    global joint_positions2
    rospy.init_node('panda_optimal_force')
    rospy.Subscriber("/panda_table/franka_state_controller/joint_states", JointState, callback1, queue_size= 2)
    rospy.Subscriber("/panda_box/franka_state_controller/joint_states", JointState, callback2, queue_size= 2)

    publish_gcmd1 = rospy.Publisher('/panda_table/panda/cmd_force_z', Float64, queue_size=10)
    publish_gcmd2 = rospy.Publisher('/panda_box/panda/cmd_force_z', Float64, queue_size=10)


    #client_panda_table = dynamic_reconfigure.client.Client("/panda_table/dynamic_reconfigure_desired_mass_param_node", timeout=30)
    #client_panda_box = dynamic_reconfigure.client.Client("/panda_box/dynamic_reconfigure_desired_mass_param_node", timeout=30)

    rate = rospy.Rate(50) # 100hz
    while not rospy.is_shutdown():
        f1, f2 = plot_polytope(joint_positions1,joint_positions2, 12.1*9.81)
        #client_panda_table.update_configuration({"desired_mass":f1/9.81, "k_p":0, "k_i":0})
        #client_panda_box.update_configuration({"desired_mass":f2/9.81, "k_p":0, "k_i":0})
        publish_gcmd1.publish(f1/9.81)
        publish_gcmd2.publish(f2/9.81)

        rate.sleep()

# class definition
if __name__ == '__main__':
    try:
        panda_optimal_force()
    except rospy.ROSInterruptException:
        pass