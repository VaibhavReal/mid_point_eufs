#!/usr/bin/env python3
import roslib
roslib.load_manifest('first_lap')

import rospy
from clustering.msg import arrofarr
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
import numpy as np
from sensor_msgs.msg import NavSatFix

initialized = False
initial_x = initial_y = 0 
earth_radius = 6378137
prev_x = prev_y = 0


markerArray = MarkerArray()


global prev_error
prev_error = 0.0


class LidarOdomNode:
    def __init__(self):
        rospy.init_node('lidar_odom_node', anonymous=True)
        self.odom_data = None
        self.prev_error_speed = 0.0
        self.prev_error_steer = 0.0
        self.prev_x = 0.0
        self.left_mapped = []
        self.right_mapped = []
        rospy.Subscriber('/robot_control/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/lidar_coordinate', arrofarr, self.lidar_callback)
        rospy.Subscriber("/gps", NavSatFix , self.gps_callback)
        self.command_pub = rospy.Publisher('/robot_control/command', AckermannDriveStamped, queue_size = 2)
    def gps_callback(self,data):
        global initialized, initial_x, initial_y, prev_x, prev_y
        if not initialized:
            initial_x ,initial_y = gps_to_local(data.latitude,data.longitude)
            initialized = True
            rospy.loginfo("Initialized GPS position")
        x, y = gps_to_local(data.latitude, data.longitude)
        self.current_x = x - initial_x
        self.current_y = y - initial_y 

    def odom_callback(self, msg):
        self.odom_data = msg
    def control_car(self):
        if self.odom_data is None:
            return

        target_x, target_y = self.midpoint
        distance_to_target = math.sqrt((target_x - self.current_x) ** 2 + (target_y - self.current_y) ** 2)

        angle_to_target = math.atan2(target_y - self.current_y, target_x - self.current_x)

        steering_error = angle_to_target - self.current_yaw
        steering_error = self.normalize_angle(steering_error)
        self.vel_input = 1.5
        kp = 2.0
        kd = 2.0
        steering_angle = 0.0
        c = 0.6
        delta1 = steering_error-self.prev_error_steer
        steering_angle = (c)*(kp*(steering_error) + kd*(delta1))

        speed_error = distance_to_target
        delta2 = speed_error-self.prev_error_speed
        speed_output = (c)*(kp*(speed_error) + kd*(delta2))

        self.prev_error_steer = steering_error
        self.prev_error_speed = speed_error
        command = AckermannDriveStamped()
        command.drive.speed = self.vel_input
        command.drive.steering_angle = max(-0.5, min(0.5, steering_angle))
        self.command_pub.publish(command)
    def lidar_callback(self, data):
        if self.odom_data is not None:
            rospy.loginfo("Current odometry position: x=%f, y=%f, z=%f",
                          self.odom_data.pose.pose.position.x,
                          self.odom_data.pose.pose.position.y,
                          self.odom_data.pose.pose.position.z)
            
            #self.current_x = self.odom_data.pose.pose.position.x
            #self.current_y = self.odom_data.pose.pose.position.y
            self.quaternion = self.odom_data.pose.pose.orientation
            _, _, self.current_yaw = self.quaternion_to_euler(self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w)
            
            self.cone_points = data.data
            self.cone_point = []
            self.left_cone_points = []
            self.right_cone_points = []
            self.left_world_points = []
            self.right_world_points = []

            for i in self.cone_points:
                self.cone_point = list(i.data)
                if self.cone_point[1] < 0 and self.cone_point[0] > 0:
                    self.right_cone_points.append(self.cone_point)
                elif self.cone_point[1] > 0 and self.cone_point[0] > 0:
                    self.left_cone_points.append(self.cone_point)

            #marker_publisher = rospy.Publisher('visualization_marker_world', MarkerArray, queue_size=10)
            self.transform_to_global(self.left_cone_points, self.left_world_points)
            #self.marker_array(self.left_cone_points, (0.0,0.0,1.0,1.0))
            self.transform_to_global(self.right_cone_points, self.right_world_points)
            #self.marker_array(self.right_cone_points, (1.0,1.0,0.0,1.0))
            #marker_publisher.publish(markerArray)
            #markerArray.markers.clear()

            for point in self.left_world_points:
                point = [round(point[0],2), round(point[1],2), round(point[2],2)]
                if is_unique(point, self.left_mapped):
                    self.left_mapped.append(point)
            
            for point in self.right_world_points:
                point = [round(point[0],2), round(point[1],2), round(point[2],2)]
                if is_unique(point, self.right_mapped):
                    self.right_mapped.append(point)

            #print('Mapped points are: ', self.mapped)
            marker_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

            self.marker_array(self.left_mapped, (0.0,0.0,1.0,1.0))
            self.marker_array(self.right_mapped, (1.0,1.0,0.0,1.0))
            marker_publisher.publish(markerArray)
            markerArray.markers.clear()

            odom_data = [self.current_x,self.current_y,self.odom_data.pose.pose.position.z]
            #odom_data = [round(odom_data[0],2), round(odom_data[1],2), round(odom_data[2],2)]
            self.left_world_points = sorted(self.left_world_points, key=lambda point : euclidean_sort(point,odom_data))
            self.right_world_points = sorted(self.right_world_points, key=lambda point : euclidean_sort(point,odom_data))

            print("left_world_points", self.left_world_points)
            print("right_world_points", self.right_world_points)

            self.left_world_points = [point for point in self.left_world_points if euclidean_sort(point,odom_data) >= 1.0]
            self.right_world_points = [point for point in self.right_world_points if euclidean_sort(point,odom_data) >= 0.7]

            if self.left_world_points and self.right_world_points:
                self.midpoint = [(self.left_world_points[0][0]+ self.right_world_points[0][0])/2, (self.left_world_points[0][1]+ self.right_world_points[0][1])/2]
                self.control_car()
            if self.left_world_points and not self.right_world_points:
                self.turning_car(math.pi)    
            if self.right_world_points and not self.left_world_points:
                self.turning_car(-math.pi) 
            self.prev_x = self.current_x
        else:
            rospy.logwarn("Received LIDAR data but no odometry data available.")
    def turning_car(self, steering_angle):
        command = AckermannDriveStamped()
        command.drive.speed = self.vel_input
        command.drive.steering_angle = steering_angle
        self.command_pub.publish(command)
    def quaternion_to_euler(self,x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return 0.0, 0.0, yaw
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    def transform_to_global(self, list, list_world):
        for local_point in list:

            local_x = local_point[0]
            local_y = local_point[1]

            global_x = self.current_x + (local_x * math.cos(self.current_yaw) - local_y * math.sin(self.current_yaw))
            global_y = self.current_y + (local_x * math.sin(self.current_yaw) + local_y * math.cos(self.current_yaw))
            list_world += [[global_x, global_y,local_point[2]]]
        
    def marker_array(self, list_world, color):
        id = 0
        count = 0
        for point in list_world:
            marker = Marker()
            marker = Marker(type=Marker.POINTS,ns='visualization_marker_array', action=Marker.ADD)
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.lifetime = rospy.Duration.from_sec(0.2)
            marker_color = ColorRGBA()
            marker_color.r = color[0]
            marker_color.g = color[1]
            marker_color.b = color[2]
            marker_color.a = color[3]
            marker.color = marker_color
            marker.pose.orientation.w = 1.0
            marker.points.append(Point(point[0], point[1], point[2]))

            markerArray.markers.append(marker)
            for m in markerArray.markers:
                m.id = id
                id += 1
            count += 1
    
def euclidean_sort(point, odom_data):
    return math.sqrt((point[0]-odom_data[0])**2 + (point[1]-odom_data[1])**2)

def is_unique(new_point, existing_points):
    for point in existing_points:
        distance = euclidean_sort(new_point, point)
        if distance < 0.9:
            return False  # Point is too close to an existing point
    return True  # Point is unique

def gps_to_local(latitude, longitude):
    x = earth_radius * longitude * math.pi / 180
    y = earth_radius * math.log(math.tan(math.pi / 4 + latitude * math.pi / 360))
    return x, y
def main():
    node = LidarOdomNode()
    rospy.spin()

if __name__ == '__main__':
    main()
