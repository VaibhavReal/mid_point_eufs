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

markerArray = MarkerArray()

class KalmanFilter:
    def __init__(self):
        # State: [x, y, vx, vy]
        self.state = np.zeros(4)  # Initial state: position and velocity
        self.P = np.eye(4)  # State covariance matrix

        # Process noise covariance matrix (how much we trust the model)
        self.Q = np.eye(4) * 0.01

        # Measurement noise covariance matrix (how much we trust the measurements)
        self.R = np.eye(2) * 0.1

        # Measurement matrix (only x, y are measured)
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])

        # Transition matrix (predict the next state)
        self.F = np.array([[1, 0, 1, 0],  # x = x + vx * dt
                           [0, 1, 0, 1],  # y = y + vy * dt
                           [0, 0, 1, 0],  # vx stays the same
                           [0, 0, 0, 1]]) # vy stays the same

    def predict(self):
        # Predict the next state (x' = F * x)
        self.state = np.dot(self.F, self.state)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, measurement):
        # Kalman Gain (K = P * H.T * (H * P * H.T + R)^-1)
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        # Update the state with the new measurement
        y = measurement - np.dot(self.H, self.state)
        self.state = self.state + np.dot(K, y)

        # Update the covariance matrix
        I = np.eye(self.P.shape[0])
        self.P = np.dot((I - np.dot(K, self.H)), self.P)

class ConeTracker:
    def __init__(self):
        self.kf_left = KalmanFilter()   # For left cone
        self.kf_right = KalmanFilter()  # For right cone

    def update_cones(self, left_cone_meas, right_cone_meas):
        # Update left cone position with new measurement
        if left_cone_meas is not None:
            self.kf_left.update(left_cone_meas)

        # Update right cone position with new measurement
        if right_cone_meas is not None:
            self.kf_right.update(right_cone_meas)

    def predict(self):
        # Predict the next position of the cones
        self.kf_left.predict()
        self.kf_right.predict()

        left_pred = self.kf_left.state[:2]  # x, y of the left cone
        right_pred = self.kf_right.state[:2]  # x, y of the right cone

        return left_pred, right_pred

class LidarOdomNode:
    def __init__(self):
        rospy.init_node('lidar_odom_node', anonymous=True)
        self.odom_data = None
        self.cone_tracker = ConeTracker()
        self.prev_error_speed = 0.0
        self.prev_error_steer = 0.0
        self.prev_x = 0.0
        rospy.Subscriber('/robot_control/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/lidar_coordinate', arrofarr, self.lidar_callback)
        self.command_pub = rospy.Publisher('/robot_control/command', AckermannDriveStamped, queue_size = 2)
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
        self.vel_input = 1.0
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
            
            self.current_x = self.odom_data.pose.pose.position.x
            self.current_y = self.odom_data.pose.pose.position.y
            self.quaternion = self.odom_data.pose.pose.orientation
            _, _, self.current_yaw = self.quaternion_to_euler(self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w)
            
            self.dx = self.current_x - self.prev_x
            print("dx :", self.dx)
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

            marker_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
            self.transform_to_global(self.left_cone_points, (0.0,0.0,1.0,1.0), self.left_world_points)
            self.transform_to_global(self.right_cone_points, (1.0,1.0,0.0,1.0), self.right_world_points)
            marker_publisher.publish(markerArray)
            markerArray.markers.clear()

            odom_data = [self.current_x,self.current_y,self.odom_data.pose.pose.position.z]
            self.left_world_points = sorted(self.left_world_points, key=lambda point : euclidean_sort(point,odom_data))
            self.right_world_points = sorted(self.right_world_points, key=lambda point : euclidean_sort(point,odom_data))

            if len(self.left_world_points) is not None:
                nearest_left_cone = [self.left_world_points[0][0], self.left_cone_points[0][1]]
            else:
                nearest_left_cone = None

            if len(self.right_world_points) is not None:
                nearest_right_cone = [self.right_world_points[0][0], self.right_cone_points[0][1]]
            else:
                nearest_right_cone = None
            # Update Kalman Filters with the new measurements
            self.cone_tracker.update_cones(nearest_left_cone, nearest_right_cone)

            # Predict next cone positions
            predicted_left_cone, predicted_right_cone = self.cone_tracker.predict()
            if len(predicted_left_cone) and len(predicted_right_cone):
                self.midpoint = [(predicted_left_cone[0]+ predicted_right_cone[0])/2, (predicted_left_cone[1]+ predicted_right_cone[1])/2]
                print(self.midpoint)
                self.control_car()
            elif len(predicted_left_cone) and not len(predicted_right_cone):
                self.turning_car(math.pi)
            elif len(predicted_right_cone) and not len(predicted_left_cone):
                self.turning_car(-math.pi)
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
    def transform_to_global(self, list, color, list_world):
        id = 0
        count =0 
        list_w = []
        for local_point in list:

            local_x = local_point[0]
            local_y = local_point[1]

            global_x = self.current_x + (local_x * math.cos(self.current_yaw) - local_y * math.sin(self.current_yaw))
            global_y = self.current_y + (local_x * math.sin(self.current_yaw) + local_y * math.cos(self.current_yaw))
            
            list_world += [[global_x, global_y,local_point[2]]]

            marker = Marker()
            marker = Marker(type=Marker.POINTS,ns='visualization_marker_array', action=Marker.ADD)
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.lifetime = rospy.Duration.from_sec(0.5)
            marker_color = ColorRGBA()
            marker_color.r = color[0]
            marker_color.g = color[1]
            marker_color.b = color[2]
            marker_color.a = color[3]
            marker.color = marker_color
            marker.pose.orientation.w = 1.0
            marker.points.append(Point(global_x, global_y, local_point[2]))

            markerArray.markers.append(marker)
            for m in markerArray.markers:
                m.id = id
                id += 1
            count += 1

def euclidean_sort(point, odom_data):
    return math.sqrt((point[0]-odom_data[0])**2 + (point[1]-odom_data[1])**2)

def main():
    node = LidarOdomNode()
    rospy.spin()

if __name__ == '__main__':
    main()


