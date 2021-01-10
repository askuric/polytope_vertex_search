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

def plot_polytope(q, direction = None,force =None):

    # calculate dk of the robot
    pose = panda.dk_position(q)
   
    # calculate force vertexes
    start = time.time()
    force_vertex, force_polytopes = panda.force_polytope_ordered(q, direction, force)  
    
    end = time.time()
    print end - start
    print force

    # construct vertex pointcloud
    pointcloud_massage = PointCloud()
    if direction is not None: 
        force_vertex = (direction).T * force_vertex
    for i in range(force_vertex.shape[1]):
        point = Point32()
        point.x = force_vertex[0,i]/500 + pose[0]
        point.y = force_vertex[1,i]/500 + pose[1]
        point.z = force_vertex[2,i]/500 + pose[2]
        pointcloud_massage.points.append(point)

    # polytop stamped message
    pointcloud_massage.header = Header()
    pointcloud_massage.header.frame_id = namespace+'panda_link0'
    pointcloud_massage.header.stamp = rospy.Time.now()
    # publish plytop
    publish_force_polytop_vertex = rospy.Publisher(namespace+'panda/force_polytope_vertex', PointCloud, queue_size=10)
    publish_force_polytop_vertex.publish(pointcloud_massage)


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
        publish_force_polytope = rospy.Publisher(namespace+'panda/force_polytope', PolygonArray, queue_size=10)
        publish_force_polytope.publish(polygonarray_message)

    
    marker = Marker()
    marker.header.frame_id = "/panda_link0"
    marker.pose.position.x =pose[0]
    marker.pose.position.y =pose[1]
    marker.pose.position.z =pose[2]
    quaternion = tf.transformations.quaternion_from_euler(0,3.14/2,0)
    #type(pose) = geometry_msgs.msg.Pose
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    marker.type = marker.ARROW
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 1
    marker.color.a = 1
    marker.scale.x = force[2]/500
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    optimal_force = rospy.Publisher('/panda_box/cmd_force', Marker, queue_size=10)
    optimal_force.publish(marker)
    """
    f_v,f_p = panda.force_polytope_ordered(q,[0,0,1])
    fmax = np.max(f_v)
    
    f_max_msg = Float64(fmax/9.81)
    publish_fmax = rospy.Publisher(namespace+'panda/max_force_z', Float64, queue_size=2)
    publish_fmax.publish(f_max_msg)
    """

# main function of the class
def panda_polytope():
    global joint_positions
    rospy.init_node('panda_polytope')

    joint_state_topic = rospy.get_param('joint_state_topic', 'joint_states')
    rospy.Subscriber(namespace+joint_state_topic, JointState, callback, queue_size= 2)

    #polytope_direction = None 
    polytope_direction = np.array([[1,0,0],[0 ,1 ,0]]) 
    normal_force = np.array([0,0,1.0]).T 

    rate = rospy.Rate(25) #Hz
    while not rospy.is_shutdown():
        normal_force[2]=normal_force[2]+0.3
        plot_polytope(joint_positions, polytope_direction, normal_force)
        rate.sleep()

# class definition
if __name__ == '__main__':
    try:
        panda_polytope()
    except rospy.ROSInterruptException:
        pass