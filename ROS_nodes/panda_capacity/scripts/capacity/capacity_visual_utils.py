import rospy
# time evaluation
import time
from sensor_msgs.msg import JointState, PointCloud 
from geometry_msgs.msg import Polygon, Point32, PolygonStamped
from jsk_recognition_msgs.msg import PolygonArray
from std_msgs.msg import Header,Float64
from visualization_msgs.msg import Marker
import tf
import numpy as np

def create_vertex_msg(force_vertex, pose, frame, scaling_factor = 500):
    pointcloud_massage = PointCloud()
    for i in range(force_vertex.shape[1]):
        point = Point32()
        point.x = force_vertex[0,i]/scaling_factor + pose[0]
        point.y = force_vertex[1,i]/scaling_factor + pose[1]
        point.z = force_vertex[2,i]/scaling_factor + pose[2]
        pointcloud_massage.points.append(point)
    
    # polytop stamped message
    pointcloud_massage.header = Header()
    pointcloud_massage.header.frame_id = frame
    pointcloud_massage.header.stamp = rospy.Time.now()
    return pointcloud_massage


def create_polytopes_msg(force_polytopes, pose, frame, scaling_factor = 500):
    polygonarray_message = PolygonArray()
    polygonarray_message.header = Header()
    polygonarray_message.header.frame_id = frame
    polygonarray_message.header.stamp = rospy.Time.now()
    for face_polygon in force_polytopes:
        polygon_massage = Polygon()
        for i in range(face_polygon.shape[1]):
            point = Point32()
            point.x = face_polygon[0,i]/scaling_factor + pose[0]
            point.y = face_polygon[1,i]/scaling_factor + pose[1]
            point.z = face_polygon[2,i]/scaling_factor + pose[2]
            polygon_massage.points.append(point)

        # polytope stamped message
        polygon_stamped = PolygonStamped()
        polygon_stamped.polygon = polygon_massage
        polygon_stamped.header = Header()
        polygon_stamped.header.frame_id = frame
        polygon_stamped.header.stamp = rospy.Time.now()
        polygonarray_message.polygons.append(polygon_stamped)
        polygonarray_message.likelihood.append(1.0)
    return polygonarray_message


def create_ellipsoid_msg(S, U, pose, frame, scaling_factor = 500):
    # calculate rotation matrix
    Rot_f = np.identity(4)
    Rot_f[0:3,0:3] = U

    marker = Marker()
    marker.header.frame_id = frame
    marker.pose.position.x = pose[0]
    marker.pose.position.y = pose[1]
    marker.pose.position.z = pose[2]
    quaternion = tf.transformations.quaternion_from_matrix( Rot_f )
    #type(pose) = geometry_msgs.msg.Pose
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    marker.type = marker.SPHERE
    marker.color.g = 0.7
    marker.color.r = 1.0
    marker.color.a = 0.5
    marker.scale.x = 2*S[0]/scaling_factor
    marker.scale.y = 2*S[1]/scaling_factor
    marker.scale.z = 2*S[2]/scaling_factor
    return marker



# definition of the four_link_solver module
if __name__ == '__main__':
    capacity_visual_utils() 
