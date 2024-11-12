#!/usr/bin/env python3
import roslib
roslib.load_manifest('first_lap')

import rospy
import tf
from geometry_msgs.msg import PointStamped
from clustering.msg import arrofarr
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA

markerArray = MarkerArray()

global prev_error
prev_error = 0.0

class LidarOdomNode:
    def __init__(self):
        rospy.init_node('lidar_odom_node', anonymous=True)
        self.odom_data = None
        self.prev_error_speed = 0.0
        self.prev_error_steer = 0.0

        rospy.Subscriber('/robot_control/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/lidar_coordinate', arrofarr, self.lidar_callback)
        self.command_pub = rospy.Publisher('/robot_control/command', AckermannDriveStamped, queue_size=2)

    def odom_callback(self, msg):
        # Store the latest odometry data
        self.odom_data = msg

    def lidar_callback(self, data):
        if self.odom_data is not None:
            rospy.loginfo("Current odometry position: x=%f, y=%f, z=%f",
                          self.odom_data.pose.pose.position.x,
                          self.odom_data.pose.pose.position.y,
                          self.odom_data.pose.pose.position.z)
            self.cone_points = data.data
            self.cone_point = []
            self.left_cone_points = []
            self.right_cone_points = []
            for i in self.cone_points:
                self.cone_point = list(i.data)
                if self.cone_point[1] < 0:
                    self.right_cone_points.append(self.cone_point)
                elif self.cone_point[1] > 0:
                    self.left_cone_points.append(self.cone_point)
            marker_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)
            transformation(self.left_cone_points, (0.0, 0.0, 1.0, 1.0))
            transformation(self.right_cone_points, (1.0, 1.0, 0.0, 1.0))
            marker_publisher.publish(markerArray)
            markerArray.markers.clear()  # Clear the markerArray after publishing
            curr_pose = [0.0, 0.0, 0.0, 0.0]
            self.left_cone_points = sorted(self.left_cone_points, key=lambda point: euclidean_sort(point, curr_pose))
            self.right_cone_points = sorted(self.right_cone_points, key=lambda point: euclidean_sort(point, curr_pose))
            self.midpoint = [(self.left_cone_points[0][0] + self.right_cone_points[0][0]) / 2,
                            (self.left_cone_points[0][1] + self.right_cone_points[0][1]) / 2]
            # self.control_car()
        else:
            rospy.logwarn("Received LIDAR data but no odometry data available.")

    def quaternion_to_euler(self, x, y, z, w):
        """Convert a quaternion into Euler angles (yaw only)."""
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return 0.0, 0.0, yaw  # Return only the yaw angle

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def transformation(list, color):
    markers_max = 100
    id = 0
    count = 0
    for i in range(len(list)):
        point_in_base_link1 = PointStamped()
        point_in_base_link1.header.frame_id = "velodyne"
        point_in_base_link1.point.x = list[i][0]
        point_in_base_link1.point.y = list[i][1]
        point_in_base_link1.point.z = list[i][2]

        marker = Marker()
        marker = Marker(type=Marker.POINTS, ns='visualization_marker_array', action=Marker.ADD)
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.lifetime = rospy.Duration.from_sec(1.0)
        marker_color = ColorRGBA()
        marker_color.r = color[0]
        marker_color.g = color[1]
        marker_color.b = color[2]
        marker_color.a = color[3]
        marker.color = marker_color
        marker.pose.orientation.w = 1.0
        marker.points.append(Point(point_in_base_link1.point.x, point_in_base_link1.point.y, point_in_base_link1.point.z))
        markerArray.markers.append(marker)
        for m in markerArray.markers:
            m.id = id
            id += 1
        count += 1

def euclidean_sort(point, odom_data):
    return math.sqrt((point[0] - odom_data[0]) ** 2 + (point[1] - odom_data[1]) ** 2)

def main():
    node = LidarOdomNode()
    rospy.spin()

if __name__ == '__main__':
    main()
