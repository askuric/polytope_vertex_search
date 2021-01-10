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

namespace = "/panda_table/"

joint_positions1 = [0,0,0,0,0,0,0]
joint_positions2 = [0,0,0,0,0,0,0]

# function drawing the marker upon receiving new joint positions
def callback1(data):
    global joint_positions1
    joint_positions1 = data.position

def callback2(data):
    global joint_positions2
    joint_positions2 = data.position

def plot_polytope(q1,q2):
    # calculate dk of the robot
    pose = panda.dk_position(q1)
    # calculate force vertexes
    start = time.time()
    force_vertex, force_polytopes = panda.force_polytope_sum_auctus(q1,q2)
    end = time.time()
    print end - start

    pointcloud_massage = PointCloud()
    for i in range(force_vertex.shape[1]):
        point = Point32()
        point.x = force_vertex[0,i]/2000 + pose[0]
        point.y = force_vertex[1,i]/2000 + pose[1]
        point.z = force_vertex[2,i]/2000 + pose[2]
        pointcloud_massage.points.append(point)

    
    polygonarray_message = PolygonArray()
    polygonarray_message.header = Header()
    polygonarray_message.header.frame_id = namespace+'panda_link0'
    polygonarray_message.header.stamp = rospy.Time.now()
    for face_polygon in force_polytopes:
        polygon_massage = Polygon()
        for i in range(face_polygon.shape[1]):
            point = Point32()
            point.x = face_polygon[0,i]/2000 + pose[0]
            point.y = face_polygon[1,i]/2000 + pose[1]
            point.z = face_polygon[2,i]/2000 + pose[2]
            polygon_massage.points.append(point)

        # polytope stamped message
        polygon_stamped = PolygonStamped()
        polygon_stamped.polygon = polygon_massage
        polygon_stamped.header = Header()
        polygon_stamped.header.frame_id = namespace+'panda_link0'
        polygon_stamped.header.stamp = rospy.Time.now()
        polygonarray_message.polygons.append(polygon_stamped)
        polygonarray_message.likelihood.append(1.0)

    # publish plytope
    publish_force_polytope = rospy.Publisher('/combined/panda/force_polytope', PolygonArray, queue_size=1)
    publish_force_polytope.publish(polygonarray_message)
    

    # polytop stamped message
    pointcloud_massage.header = Header()
    pointcloud_massage.header.frame_id = namespace+'panda_link0'
    pointcloud_massage.header.stamp = rospy.Time.now()
    # publish plytop
    publish_force_polytop_vertex = rospy.Publisher('/combined/panda/force_polytope_vertex', PointCloud, queue_size=1)
    publish_force_polytop_vertex.publish(pointcloud_massage)

    #fmax = panda.max_force_in_direction(q1,[0,0,1])
    #f_max_msg = Float64(fmax)
    #publish_fmax = rospy.Publisher(namespace+'panda/max_force', Float64, queue_size=10)
    #publish_fmax.publish(f_max_msg)


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