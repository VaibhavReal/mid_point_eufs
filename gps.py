#!/usr/bin/env python3
import roslib
roslib.load_manifest('first_lap')

import rospy
from sensor_msgs.msg import NavSatFix
import math

rospy.init_node('gps_to_local_converter', anonymous=True)

initialized = False
initial_x = initial_y = 0 
earth_radius = 6378137
prev_x = prev_y = 0
prev_time = rospy.Time.now()

def gps_callback(data):
    global initialized, initial_x, initial_y, prev_x, prev_y , prev_time
    if not initialized:
        initial_x ,initial_y = gps_to_local(data.latitude,data.longitude)
        initialized = True
        rospy.loginfo("Initialized GPS position")
    x, y = gps_to_local(data.latitude, data.longitude)
    x, y = x - initial_x, y - initial_y 
    time = rospy.Time.now()
    dt = (time - prev_time).to_sec()
    print("Time:", x, y)
    vx = (x-prev_x)/dt
    vy = (y-prev_y)/dt
    rospy.loginfo("Local X: {}, Local Y: {} , Vx = {}, Vy = {}".format(x, y,vx,vy))
    prev_x, prev_y = x,y
    prev_time = time

def gps_to_local(latitude, longitude):
    x = earth_radius * longitude * math.pi / 180
    y = earth_radius * math.log(math.tan(math.pi / 4 + latitude * math.pi / 360))
    return x, y

def listener():
    rospy.Subscriber("/gps", NavSatFix, gps_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass