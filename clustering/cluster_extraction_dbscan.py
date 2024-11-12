#!/usr/bin/env python3

import open3d as o3d
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import pcl
from clustering.msg import arr, arrofarr

# pub = rospy.Publisher('/cone_pose', PointCloud2, queue_size=10)
pub = rospy.Publisher('/clusters', arrofarr, queue_size=10)

def main(msg):
    

    #Converting from PointCloud2 msg type to o3d pointcloud
    pc_data = []
    for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        pc_data.append([point[0], point[1], point[2]])
    if pc_data:
        pc_data = np.array(pc_data)
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_data))


    #labels is an array which at every index has a number associated with a cluster of that number
    # so '2' at index '3' in labels signifies that the point in pointcloud at index 3 belongs to cluster number 2
    labels = np.array(cloud.cluster_dbscan(eps=0.30, min_points=3, print_progress=False))
    max_label = labels.max()
    #print(f"point cloud has {max_label + 1} clusters")

    rospy.loginfo('Performing dbscan clustering')
    rospy.loginfo(max_label+1)

    
    # appending the cluster number to each point: [x_coordinate, y_coordinate, z_coordinate, cluster_number] and adding these points to a list
    #Publishing the said list
    # (cluster number starts from -1. But -1 corrosponds to noise. So implemented a check in next node to not consider the -1 cluster)
    points_with_clusters = arrofarr()
    for i in range(len(cloud.points)):
        temp = arr()
        temp.data = cloud.points[i].tolist()
        temp.data.append(float((labels[i])))
        points_with_clusters.data.append(temp)

    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "velodyne"
    points_with_clusters.header = header
    rospy.loginfo('Publishing cluster_indices to /clusters')
    pub.publish(points_with_clusters)
    
    

    # #dict1 stores cluster_no. as key and list of indices of points in that cluster as values
    # #dict1 starts with key number -1 and the values of key -1 corresponds to noise
    # k=0
    # dict1 = {}
    # for i in range(len(np.asarray(cloud.points))):
    #     if(labels[k] in dict1):
    #         dict1[labels[k]].append(i)
    #     else:
    #         dict1[labels[k]] = []
    #         dict1[labels[k]].append(i)
    #     k+=1
    

    # #cluster_indices is a list consisting of lists. eg: the sublist at index 2 contains *index* of all those points in cluster no. 2
    # #distofcluster contains 2 at index 3: so distance of cluster 3 is 2 units
    # cluster_indices = []indices_msg
    # distofcluster = []
    # tempsumofdist = 0
    # for i in dict1:
    #     if (i!=-1):
    #         cluster_indices.append(dict1[i])
    #         for j in dict1[i]:
    #             tempsumofdist += (pc_data[j][0]**2 + pc_data[j][1]**2 + pc_data[j][2]**2)**0.5
    #         avgdist  = tempsumofdist/len(dict1[i])
    #         distofcluster.append(avgdist)
    #         tempsumofdist = 0

    # # performing a check if clusters are valid
    # cone_cluster_indices = []
    # heightcone = 0.325
    # widthcone = 0.228
    
    # # in radians
    # #for velodyne v16 (PUCK)
    # horizontalres = 0.00335103
    # verticalres = 0.03272494
    # for i in range(len(cluster_indices)):
    #     d = distofcluster[i]
    #     expectedpoints = 0.125*(1/d)*(1/d)* heightcone * widthcone * (1/math.tan(verticalres/2))* (1/math.tan(horizontalres/2))
    #     if(len(cluster_indices[i])>=expectedpoints-5 and len(cluster_indices[i])<=expectedpoints+5):
    #         cone_cluster_indices.append(cluster_indices[i])

    # print(len(cluster_indices))
    # print(len(cone_cluster_indices))

if __name__ == "__main__":
    rospy.init_node("cluster_extraction_dbscan")
    rospy.Subscriber("/no_ground_cloud", PointCloud2, main)

    # Spin to keep the node alive
    rospy.spin()
