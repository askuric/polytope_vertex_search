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
def draw_ellipsoids(q):

    # calculate dk of the robot
    pose = panda.dk_position(q)
    
    # calculate dk of the robot.
    manipulability_v = panda.manipulability_velocity(q)
    Rot_v = np.identity(4)
    Rot_v[0:3,0:3] = manipulability_v[1]
    S = manipulability_v[0]

    marker = Marker()
    marker.header.frame_id = namespace+"panda_link0"
    marker.pose.position.x = pose[0]
    marker.pose.position.y = pose[1]
    marker.pose.position.z = pose[2]
    quaternion =  tf.transformations.quaternion_from_matrix( Rot_v)
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    marker.type = marker.SPHERE
    marker.color.g = 1.0
    marker.color.r = 0.7
    marker.color.a = 0.5
    marker.scale.x = 2*S[0]/5
    marker.scale.y = 2*S[1]/5
    marker.scale.z = 2*S[2]/5

    publish_manip1 = rospy.Publisher(namespace+'panda/manip_velocity', Marker, queue_size=10)
    publish_manip1.publish(marker)

    # calculate manipulability
    manipulability_f = panda.manipulability_force(q)
    Rot_f = np.identity(4)
    Rot_f[0:3,0:3] = manipulability_f[1]
    S = manipulability_f[0]
    marker = Marker()
    marker.header.frame_id = namespace+"panda_link0"
    marker.pose.position.x = pose[0]
    marker.pose.position.y = pose[1]
    marker.pose.position.z = pose[2]
    quaternion = tf.transformations.quaternion_from_matrix( Rot_f)
    #type(pose) = geometry_msgs.msg.Pose
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    marker.type = marker.SPHERE
    marker.color.g = 0.7
    marker.color.r = 1.0
    marker.color.a = 0.5
    marker.scale.x = 2*S[0]/500
    marker.scale.y = 2*S[1]/500
    marker.scale.z = 2*S[2]/500

    publish_manip1 = rospy.Publisher(namespace+'panda/manip_force', Marker, queue_size=10)

    publish_manip1.publish(marker)

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
