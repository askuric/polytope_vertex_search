#!/usr/bin/env python
import rospy
import math
import tf
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
# robot module import
import panda_solver as panda
import numpy as np


def show_marker_dk(pose):
    marker = Marker()
    marker.header.frame_id = "panda_link0"
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = pose[0]
    marker.pose.position.y = pose[1]
    marker.pose.position.z = pose[2]
    quaternion = tf.transformations.quaternion_from_euler(0,0,0)
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    marker.type = marker.CUBE
    marker.color.g = 1.0
    marker.color.a = 0.5
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    marker_dk = rospy.Publisher('/panda/direct_kinematics', Marker, queue_size=10)
    marker_dk.publish(marker)
    
def show_marker_ik(pose):

    marker = Marker()
    marker.header.frame_id = "panda_link0"
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = pose[0]
    marker.pose.position.y = pose[1]
    marker.pose.position.z = pose[2]
    quaternion = tf.transformations.quaternion_from_euler(0,0,0)
    #type(pose) = geometry_msgs.msg.Pose
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    marker.type = marker.CUBE
    marker.color.r = 1.0
    marker.color.a = 0.5
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    marker_ik = rospy.Publisher('/panda/inverse_kinematics', Marker, queue_size=10)
    marker_ik.publish(marker)
    

def callback(data):
    global target_pose 
    target_pose[:3,3] = [[data.x],[data.y],[data.z]]

def panda_inverse_kinematics():

    rospy.init_node('panda_inverse_kinematics', anonymous=True)
    publish_states = rospy.Publisher("/joint_states", JointState, queue_size=10)
    rospy.Subscriber("/goto", Pose, callback)

    rate = rospy.Rate(10) # 10hz sampling freq
    
    #initial state
    q = [0, 0, 0, 0, 0, 0, 0]
    # target pose received from /goto message
    global target_pose 
    target_pose = panda.forward(q)
    while not rospy.is_shutdown():

        # jointstates message construct
        joints = JointState()
        joints.header = Header()
        joints.header.stamp = rospy.Time.now()
        joints.name = ["panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7"]
        joints.effort = [0, 0, 0, 0, 0, 0, 0]
        joints.velocity = [0, 0, 0, 0, 0, 0, 0]

        # iterative solution to inverse kinematics
        q = panda.ik_solve(target_pose, q, 1)
        joints.position = q

        # publish the messqes
        publish_states.publish(joints)

        show_marker_dk(panda.dk_position(q))
        show_marker_ik(target_pose)

        # wait for next sampling time 
        rate.sleep()

if __name__ == '__main__':
    try:
        panda_inverse_kinematics()
    except rospy.ROSInterruptException:
        pass