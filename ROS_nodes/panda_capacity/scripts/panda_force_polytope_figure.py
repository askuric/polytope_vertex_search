#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState, PointCloud 
from geometry_msgs.msg import Polygon, Point32, PolygonStamped
from jsk_recognition_msgs.msg import PolygonArray
from std_msgs.msg import Header,Float64
from visualization_msgs.msg import Marker
import tf
# robot module import
import panda_solver as panda
# time evaluation
import time
#arguments
import sys

#some math
import numpy as np

namespace = rospy.get_namespace()

joint_positions = [0,0,0,0,0,0,0]

# function receiveing the new joint positions
def callback(data):
    global joint_positions
    joint_positions = data.position

def plot_polytope_plane(q, plane):

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

    # calculate force vertexes
    force_vertex, force_polytopes = panda.force_polytope_ordered(q,direction)
    
    # draw polytope in only for 2D and 3D
    if force_polytopes[0].shape[0] > 1:
        polygonarray_message = PolygonArray()
        polygonarray_message.header = Header()
        polygonarray_message.header.frame_id = namespace+'panda_link0'
        polygonarray_message.header.stamp = rospy.Time.now()
        for face_polygon in force_polytopes:
            polygon_massage = Polygon()
            if direction is not None: 
                face_polygon = (direction).T * face_polygon
            for i in range(face_polygon.shape[1]):
                point = Point32()
                point.x = face_polygon[0,i]/500 + pose[0]
                point.y = face_polygon[1,i]/500 + pose[1]
                point.z = face_polygon[2,i]/500 + pose[2] 
                polygon_massage.points.append(point)

            # polytope stamped message
            polygon_stamped = PolygonStamped()
            polygon_stamped.polygon = polygon_massage
            polygon_stamped.header = Header()
            polygon_stamped.header.frame_id = namespace+'panda_link0'
            polygon_stamped.header.stamp = rospy.Time.now()
            polygonarray_message.polygons.append(polygon_stamped)
            polygonarray_message.likelihood.append(1.0)

        # publish plytop
        publish_force_polytope = rospy.Publisher(namespace+"polytope_"+plane, PolygonArray, queue_size=10)
        publish_force_polytope.publish(polygonarray_message)

# main function of the class
def panda_polytope():
    global joint_positions
    rospy.init_node('panda_polytope')

    joint_state_topic = rospy.get_param('joint_state_topic', 'joint_states')
    rospy.Subscriber(namespace+joint_state_topic, JointState, callback, queue_size= 2)

    polytope_direction = None #np.array([[0 ,0 ,1]]) 

    rate = rospy.Rate(35) #Hz
    while not rospy.is_shutdown():
        plot_polytope_plane(joint_positions, 'xy')
        plot_polytope_plane(joint_positions, 'xz')
        plot_polytope_plane(joint_positions, 'yz')
        rate.sleep()

# class definition
if __name__ == '__main__':
    try:
        panda_polytope()
    except rospy.ROSInterruptException:
        pass