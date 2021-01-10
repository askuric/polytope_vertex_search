#!/usr/bin/env python
import rospy
import tf
import numpy as np
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
# robot module import
import panda_solver as panda


namespace = rospy.get_namespace()

namespace = rospy.get_namespace()

joint_positions = [0,0,0,0,0,0,0]

# function receiveing the new joint positions
def callback(data):
    global joint_positions
    joint_positions = data.position

# function drawing the marker upon receiving new joint positions
def draw_ellipsoids_plane(q, plane):

    # calculate dk of the robot
    pose = panda.dk_position(q)

    if plane == 'xy':
        direction = np.array([[1 ,0 ,0],[0 ,1 ,0]])
        pose[2] = 0
    elif plane == 'xz':
        direction = np.array([[1 ,0 ,0],[0 ,0 ,1]])
        pose[1] = 1
    elif plane == 'yz':
        direction = np.array([[0 ,1 ,0],[0 ,0 ,1]])
        pose[0] = -0.6

    # calculate manipulability
    manipulability_f = panda.manipulability_force(q, direction)
    S = manipulability_f[0]
    U = manipulability_f[1]
    marker = Marker()
    marker.header.frame_id = namespace+"panda_link0"
    marker.pose.position.x = pose[0]
    marker.pose.position.y = pose[1]
    marker.pose.position.z = pose[2]
    angle = np.arctan2(U[0,1],U[0,0])
    if plane == 'xy':
        quaternion = tf.transformations.quaternion_from_euler(0,0,angle)
    elif plane == 'xz':
        quaternion = tf.transformations.quaternion_from_euler(0,-angle,0)
    elif plane == 'yz':
        quaternion = tf.transformations.quaternion_from_euler(angle,0,0)
    
    #type(pose) = geometry_msgs.msg.Pose
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    marker.type = marker.SPHERE
    marker.color.g = 0.7
    marker.color.r = 1.0
    marker.color.a = 0.5
    if plane == 'xy':
        marker.scale.x = 2*S[0]/500
        marker.scale.y = 2*S[1]/500
        marker.scale.z = 0.01
    elif plane == 'xz':
        marker.scale.x = 2*S[0]/500
        marker.scale.z = 2*S[1]/500
        marker.scale.y = 0.01
    elif plane == 'yz':
        marker.scale.y = 2*S[0]/500
        marker.scale.z = 2*S[1]/500
        marker.scale.x = 0.01

    publish_manip1 = rospy.Publisher(namespace+'panda/manip_force_'+plane, Marker, queue_size=10)

    publish_manip1.publish(marker)

# main funciton of the class
def panda_manipulability():
    global joint_positions
    rospy.init_node('panda_manipulability', anonymous=True)
    rospy.Subscriber(namespace+"joint_states", JointState, callback)


    rate = rospy.Rate(35) # 10hz
    while not rospy.is_shutdown():
        draw_ellipsoids_plane(joint_positions,'xy')
        draw_ellipsoids_plane(joint_positions,'xz')
        draw_ellipsoids_plane(joint_positions,'yz')
        rate.sleep()

# class definition
if __name__ == '__main__':
    try:
        panda_manipulability()
    except rospy.ROSInterruptException:
        pass
