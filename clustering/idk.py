#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import message_filters
from clustering.msg import arr, arrofarr

def main(pcd_msg, indices_msg):
    #Converting from PointCloud2 msg type to o3d pointcloud
    pc_data = []
    for point in point_cloud2.read_points(pcd_msg, field_names=("x", "y", "z"), skip_nans=True):
        pc_data.append([point[0], point[1], point[2]])
    if pc_data:
        pc_data = np.array(pc_data)
        print(pc_data)
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_data))

    print(indices_msg)
        

if __name__ == "__main__":
    rospy.init_node("postprocessing")
    pcd_sub = message_filters.Subscriber("/no_ground_cloud", PointCloud2)
    indices_sub = message_filters.Subscriber("/cluster_indices", arrofarr)
    ts = message_filters.TimeSynchronizer([pcd_sub, indices_sub], 10)
    ts.registerCallback(main)

    # Spin to keep the node alive
    rospy.spin()