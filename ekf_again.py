#!/usr/bin/env python3
import roslib
roslib.load_manifest('first_lap')

import rospy
from clustering.msg import arrofarr
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import ColorRGBA
import numpy as np
from tf.transformations import quaternion_from_euler,euler_from_quaternion


class ExtendedKalman():
    def __init__(self):
        rospy.init_node('extended_kalman_filter', anonymous=True)
        self.odom_data = None
        self.initialized = False
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.prev_omega = 0.0
        self.state = [[self.prev_x], [self.prev_y], [self.prev_theta]]
        self.p_0 = np.zeroes(self.x_hat.shape, self.x_hat.shape)
        self.time = 0.1
        self.Q = np.zeroes(self.x_hat.shape, self.x_hat.shape)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.pub = rospy.Publisher('/extended_kalman_filter/odom', Odometry, queue_size=2)
        rospy.Subscriber('/lidar_coordinate', arrofarr, self.lidar_callback)
    def imu_callback(self, msg):
        (_, _ , self.curr_yaw) = euler_from_quaternion(msg.orientation)
    def predict(self):
        del_theta = heading_rate * self.time
        del_s = self.vx * self.time
        f_k = [[self.prev_x + (del_s * math.cos(self.prev_theta + del_theta/2))],
                [self.prev_y + (del_s * math.cos(self.prev_theta + del_theta/2))],
                [self.prev_theta + del_theta]]
        
    def lidar_callback(self, msg):
        if self.initialized == False:
            initial = self.curr_yaw
            self.initialized = True
        
        

